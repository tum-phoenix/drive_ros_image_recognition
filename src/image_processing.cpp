#include <drive_ros_image_recognition/image_processing.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pluginlib/class_list_macros.h>
#include <math.h>
#include "ros/ros.h"
#include <drive_ros_msgs/Obstacle.h>
#include <drive_ros_image_recognition/image_processing.h>

namespace drive_ros_image_recognition {

ImageProcessing::ImageProcessing(ros::NodeHandle nh, ros::NodeHandle pnh, bool nodelet) : nodelet_(
                                                                                            nodelet) //initialisation list?
{
  image_transport::ImageTransport it(pnh);
  img_sub_ = it.subscribe("img_in", 1, &ImageProcessing::imageCallback, this);
#ifdef DRAW_DEBUG
  cv::namedWindow("detected lines", CV_WINDOW_NORMAL);
#endif
  reconfigure_server_.setCallback(boost::bind(&ImageProcessing::reconfigureCallback, this, _1, _2));
}

void ImageProcessing::imageCallback(
    const sensor_msgs::ImageConstPtr &msg) //sensor_msgs: messages for commonly used sensors eg. cameras
{
  //cv_bridge converts between ROS Image messages and OPENCV pointers
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg); //converts sensor_msgs::Image to OpenCV-compatible CvImage
  }
  catch (cv_bridge::Exception &e) {
    ROS_ERROR_STREAM_NAMED("ImageProcessing", "cv_bridge exception: " << e.what());
    return;
  }


  cv::Mat img_in = cv_ptr->image; //bgr

  //apply blur
  cv::GaussianBlur(img_in, img_in, cv::Size(15, 15), 0, 0);

  cv::Mat dst, cdst;
  cv::Canny(img_in, dst, canny_param1_, canny_param2_); //result from Canny edge detection in dst
#ifdef DRAW_DEBUG
  cv::cvtColor(dst, cdst, CV_GRAY2BGR); // change colors for debug image
#endif
  //probabilistic Hough
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(dst, lines, 1, (CV_PI / 180), hough_threshold_, hough_min_line_length_, hough_max_line_gap_);

  int counter = 0;
  float bin_size = 20.f;
  std::vector<int> binned_lines((int)(180.f/bin_size));

  for (const cv::Vec4i &l : lines) {
#ifdef DRAW_DEBUG
    cv::line(cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 1, CV_AA);
#endif
    if (std::pow(l[3]-l[1], 2)+std::pow(l[2]-l[0], 2) < squared_length_threshold_) {
      float angle = cv::fastAtan2(l[3]-l[1], l[2]-l[0]);
      angle = angle > 180.f ? angle-180.f : angle;
//      binned_lines((int)(angle/bin_size))++;
    }
  }


//  for (const cv::Vec4i l : lines){
//    float x1 = (float) l[0], y1 = (float) l[1], x2 = (float) l[2], y2 = (float) l[3];

//    if (x2-x1 <=20 && y2-y1<=20) { //compare short lines
//      float gradient = (y2 - y1) / (x2 - x1);

//      float current_angle = atan(gradient);

//      //compare with previously stored angles
//      int count = 0;
//      for (const float &angle: angles) {
//        //take the absolute of difference
//        float difference = abs(current_angle - angle);

//        //if not acute angle subtract value from pi radians
//        if (difference > (M_PI_2)) {
//          difference = M_PI - difference;
//        }

//        if (difference >= (((float) 25 / (float) 180) * M_PI) &&
//            difference <= (((float) 30 / (float) 180) * M_PI)) {
//          count++;

//          if (count>=4) {
//            possibleLines.push_back(angle);
//#ifdef DRAW_DEBUG
//            cv::line(cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 255, 0), 3,
//                CV_AA);
//#endif
//          }
//        }
//      }
//    }
//  }

//  //now check if at least 2 lines should be almost parallel among the possibleLines
//  for(const float &angle : possibleLines){
//    for(std::vector<float>::iterator it3 = possibleLines.begin(); it3!=possibleLines.end(); ++it3){
//      float compare = *it3;
//      if (&compare != &angle){
//        float diff = abs(angle-compare);
//        if (diff > M_PI_2){
//          diff = M_PI - diff;
//        }
//        if (diff<=0.1){
//          counter++;
//        }
//      }
//    }
//  }

  std::cout<<"new image\n";
  if (counter >= 5) {
    //publish barred area message
    ros::NodeHandle n;
    std::cout << "barred area detected\n";
    ros::Publisher pub = n.advertise<drive_ros_msgs::Obstacle>("barred", 1000);
    drive_ros_msgs::Obstacle msg;
    //set message fields

    /*
                 * msg.centroid_pose = ;
                pub.publish(msg);
                 */
  }

  ROS_INFO_STREAM("DETECTED NUMBER OF LINES: "<<counter);
#ifdef DRAW_DEBUG
  if (!nodelet_)
    cv::imshow("detected lines", cdst);
#endif
  cv::waitKey(0);
}

void ImageProcessing::reconfigureCallback(drive_ros_image_recognition::BarredAreaDetectionConfig &config,
                                          uint32_t level)
{
  canny_param1_ = config.canny_param1;
  canny_param2_ = config.canny_param2;

  hough_threshold_ = config.hough_threshold;
  hough_min_line_length_ = config.hough_min_line_length;
  hough_max_line_gap_ = config.hough_max_line_gap;
}

void ImageProcessingNodelet::onInit() {
  img_proc_.reset(new ImageProcessing(getNodeHandle(), getPrivateNodeHandle(), true));
}

} // namespace drive_ros_image_recognition

PLUGINLIB_EXPORT_CLASS(drive_ros_image_recognition::ImageProcessingNodelet, nodelet::Nodelet)

