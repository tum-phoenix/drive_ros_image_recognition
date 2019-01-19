#include <drive_ros_image_recognition/image_processing.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pluginlib/class_list_macros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>


namespace drive_ros_image_recognition {

ImageProcessing::ImageProcessing(ros::NodeHandle nh, ros::NodeHandle pnh, bool nodelet) : nodelet_(nodelet)
{
  image_transport::ImageTransport it(pnh);
  img_sub_ = it.subscribe("img_in", 1, &ImageProcessing::imageCallback, this);
}

bool ImageProcessing::detectLaneLines(cv::Mat image, cv::Vec4i& l1, cv::Vec4i& l2){
    cv::Mat blurImg;
    cv::GaussianBlur(image, blurImg, cv::Size(11,11), 11);

    cv::Mat dst, edges;
    cv::Canny(blurImg, edges, 255, 255);
    image.copyTo(dst, edges);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );

    cv::Mat hough;
    cv::cvtColor(image, hough, CV_GRAY2BGR);
    float negSlope = 0;
    float posSlope = 0;
    int n = 0, p = 0;

    cv::Point right = cv::Point(dst.cols + 1, dst.rows + 1);
    cv::Point left = cv::Point(-1, -1);

    float leftSlope = 1;
    float rightSlope = -1;

    int rc = true;

    for(unsigned int i = 0; i < lines.size(); i++ ){
      cv::Vec4i l = lines[i];
      cv::line( hough, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
      float m = (l[3] - l[1]) * 1.0 / (l[2] - l[0]);
      //std::cout << m << std::endl;

      if (m > 0){
          if (l[0] < right.x){
              right = cv::Point(l[0], l[1]);
              rightSlope = m;
          }

          posSlope += m;
          p++;
      } else {
          if (l[0] > left.x){
              left = cv::Point(l[0], l[1]);
              leftSlope = m;
          }
          negSlope += m;
          n++;
      }
    }


    float meanNegSlope = 0;
    float meanPosSlope = 0;
    float varNegSlope = 0;
    float varPosSlope = 0;

    float b1 = 0, b2 = 0;

    if (p != 0){
        meanPosSlope = posSlope / p;
    }

    if (rightSlope > 0){
        b1 = right.y - rightSlope*right.x;
        l1 = cv::Vec4i(-b1/rightSlope, 0, (dst.rows - b1 - 1)/rightSlope, dst.rows - 1);
        cv::line(hough, cv::Point(l1[0], l1[1]), cv::Point(l1[2], l1[3]), cv::Scalar(0,255, 0), 3);
    } else {
        rc = false;
    }

    if (n != 0){
        meanNegSlope = negSlope / n;
    }

    if (leftSlope < 0){
        b2 = left.y - leftSlope*left.x;
        l2 = cv::Vec4i(-b2/leftSlope, 0, (dst.rows - b2 - 1)/leftSlope, dst.rows - 1);
        cv::line(hough, cv::Point(l2[0], l2[1]), cv::Point(l2[2], l2[3]), cv::Scalar(0,255, 0), 3);
    } else {
        rc = false;
    }

    for (size_t i = 0; i < lines.size(); i++){
        cv::Vec4i l = lines[i];
        float m = (l[3] - l[1]) * 1.0 / (l[2] - l[0]);
        if (m > 0){
            varPosSlope += (m - meanPosSlope)*(m - meanPosSlope);
        } else {
            varNegSlope += (m - meanNegSlope)*(m - meanNegSlope);
        }
    }

    if (p > 1){
       varPosSlope /= (p - 1);
    }

    if (n > 1){
        varNegSlope /= (n - 1);
    }

    //std::cout << "Neg slope mean " << meanNegSlope << " variance " << varNegSlope << std::endl;
    //std::cout << "Pos slope mean " << meanPosSlope << " variance " << varPosSlope << std::endl;
    //cv::imshow("Hough", hough);

    return rc;
}

void ImageProcessing::extractRelevantRegion(cv::Mat img_in, cv::Mat& mask, cv::Mat& img_out, const cv::Vec4i l1, const cv::Vec4i l2){
    img_in.copyTo(img_out);
    mask = cv::Mat(img_in.rows, img_in.cols, CV_8U, cv::Scalar::all(255));

    float m1 = (l1[3] - l1[1])*1.0 / (l1[2] - l1[0]);
    float m2 = (l2[3] - l2[1])*1.0 / (l2[2] - l2[0]);

    float b1 = l1[1] - m1*l1[0];
    float b2 = l2[1] - m2*l2[0];

    for (int y = 0; y < img_in.rows; y++){
        for (int x = 0; x < img_in.cols; x++){
            float y_l1 = x*m1 + b1;
            float y_l2 = x*m2 + b2;
            if (y < y_l1 || y < y_l2 || y > 0.75*img_in.rows){
                img_out.at<uchar>(y,x) = 0;
                mask.at<uchar>(y,x) = 0;
            }
        }
    }

    //cv::imshow("Mask", mask);
}

bool ImageProcessing::findStartLineRANSAC(std::vector<cv::Point2f> corners, cv::Vec4i l1, cv::Vec4i l2,
                                          cv::Vec4i& startline, float& error){
    if (corners.size() < 2){
        return false;
    }

    int numIterations = 1000;
    int n = 0;
    float threshold = 2;
    std::vector<cv::Point2f> bestModel;
    float bestError = FLT_MAX;
    float bestSlope = 0;
    float bestIntercept = 0;

    std::srand(std::time(0));
    while (n < numIterations){
        std::vector<cv::Point2f> points;
        size_t index1 = rand() % corners.size();
        size_t index2 = index1;
        while(index1 == index2) {
            index2 = rand() % corners.size();
        }

        float m = (corners[index1].y - corners[index2].y) / (corners[index1].x - corners[index2].x);
        float b = corners[index1].y - m*corners[index1].x;

        std::vector<cv::Point2f> model;
        model.push_back(corners[index1]);
        model.push_back(corners[index2]);

        float error = 0;
        for (unsigned int i = 0; i < corners.size(); i++){
            if (i == index1 || i == index2){
                continue;
            }

            float currError = std::abs(corners[i].y - m*corners[i].x - b);
            if (currError < threshold){
                model.push_back(corners[i]);
                error += currError;
            }
        }

        if (model.size() > bestModel.size()){
            float relError = error / model.size();
            if (relError < bestError){
                bestModel = model;
                bestError = relError;
                bestSlope = m;
                bestIntercept = b;
            }
        }
        n++;
    }
    //std::cout << "Bestmodel size " << bestModel.size() << " error " << bestError << std::endl;
    cv::fitLine(bestModel, startline, CV_DIST_L1, 0, 0.01, 0.01);
    error = bestError;

    return (bestModel.size() > 3);
}

bool ImageProcessing::detectStartLine(cv::Mat img_out, std::vector<cv::Point2f> corners, cv::Vec4i l1, cv::Vec4i l2){
    cv::Vec4i line = cv::Vec4i(0,0,0,0);
    float error = FLT_MAX;
    if(!findStartLineRANSAC(corners, l1, l2, line, error)){
        return false;
    }

    float m1 = (l1[3] - l1[1])*1.0 / (l1[2] - l1[0]);
    float m2 = (l2[3] - l2[1])*1.0 / (l2[2] - l2[0]);

    float b1 = l1[1] - m1*l1[0];
    float b2 = l2[1] - m2*l2[0];

    float m_line = line[1]*1.0 / line[0];
    float b_line = line[3] - m_line*line[2];

    if (!std::isfinite(m_line)){
        return false;
    }

    float x_intercept_line1 = (b_line - b1) / (m1 - m_line);
    float y_intercept_line1 = m1*x_intercept_line1 + b1;

    float x_intercept_line2 = (b_line - b2) / (m2 - m_line);
    float y_intercept_line2 = m2*x_intercept_line2 + b2;

    if (x_intercept_line1 < 0 || x_intercept_line1 >= img_out.cols ||
        y_intercept_line1 < 0 || y_intercept_line1 >= img_out.rows ||
        x_intercept_line2 < 0 || x_intercept_line2 >= img_out.cols ||
        y_intercept_line2 < 0 || y_intercept_line2 >= img_out.rows){
        return false;
    }

    bool white = false;
    bool black = false;
    int numSwitch = 0, w = 0, b = 0;

    int start = std::min(x_intercept_line1, x_intercept_line2);
    int end = std::max(x_intercept_line1, x_intercept_line2);

    float meanGapsBlack = 0, meanGapsWhite = 0, currGap = 0;

    for (int x = start; x <= end; x+=2){
        float y = m_line*x + b_line;
        if (img_out.at<uchar>(y,x) < 100){
            if (white){
                numSwitch = numSwitch + 1;

                meanGapsWhite += currGap;
                currGap = 0;
                white = false;
            }

            black = true;
            b++;
        }

        if (img_out.at<uchar>(y,x) >= 200){
            if (black){
                numSwitch = numSwitch + 1;

                meanGapsBlack += currGap;
                currGap = 0;
                black = false;
            }

            white = true;
            w++;
        }

        currGap++;
    }

    if (w > 0){
        meanGapsWhite /= w;
    }

    if (b > 0){
        meanGapsBlack /= b;
    }

//    std::cout << "Number of switches: " << numSwitch << " meangapsBlack: " << meanGapsBlack << " ..." <<
//                                                        " meangapsWhite: " << meanGapsWhite << " ..." <<std::endl;

//    cv::Mat copy;
//    cv::cvtColor(img_out, copy, CV_GRAY2BGR);

//    cv::line(copy,  cv::Point(x_intercept_line1, y_intercept_line1),
//                    cv::Point(x_intercept_line2, y_intercept_line2), cv::Scalar(255, 0, 0), 2);

//    cv::imshow("Line", copy);

    return numSwitch >= 10;
    //return true;
}

void ImageProcessing::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED("ImageProcessing", "cv_bridge exception: "<<e.what());
    return;
  }

  // example content: display incoming image
  // (NOTE: default OpenCV display functions don't work with nodelets as they are not suited for multi-threaded applications)
  if (!nodelet_)
  {
    cv::Mat img_in = cv_ptr->image;
    cv::Vec4i l1, l2;
    if (detectLaneLines(img_in, l1, l2)){
        cv::Mat mask, img_out;
        extractRelevantRegion(img_in, mask, img_out, l1, l2);

        int maxCorners = 20;
        std::vector<cv::Point2f> corners;
        cv::goodFeaturesToTrack(img_in, corners, maxCorners, 0.01, 10, mask, 3);

        if (detectStartLine(img_out, corners, l1, l2)){
            std::cout << "Start line is found!" << std::endl;
        }
        cv::cvtColor(img_out, img_out, CV_GRAY2BGR);
        for (unsigned int i = 0; i < corners.size(); i++){
            cv::circle(img_out, corners[i], 2, cv::Scalar(255, 0, 0), -1);
        }
        //cv::imshow("Output image", img_out);
    }
    cv::namedWindow("Incoming image", CV_WINDOW_NORMAL);
    cv::imshow("Incoming image", img_in);
    cv::waitKey(1);
  }
}

void ImageProcessingNodelet::onInit()
{
  img_proc_.reset(new ImageProcessing(getNodeHandle(), getPrivateNodeHandle(), true));;
}

} // namespace drive_ros_image_recognition

PLUGINLIB_EXPORT_CLASS(drive_ros_image_recognition::ImageProcessingNodelet, nodelet::Nodelet)

