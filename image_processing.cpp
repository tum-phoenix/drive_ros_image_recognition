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


        if (!nodelet_) {

            cv::Mat img_in = cv_ptr->image; //bgr

            //apply blur
            cv::GaussianBlur(img_in, img_in, cv::Size(15, 15), 0, 0);

            //crop image
            cv::Rect rect(10, 10, 500, 1000);
            cv::Mat croppedImage = img_in(rect);
            croppedImage.copyTo(img_in);

            cv::Mat dst, cdst;
            cv::Canny(img_in, dst, 50, 200, 3); //result from Canny edge detection in dst
            //dst = img_in;
            cv::cvtColor(dst, cdst, CV_GRAY2BGR); //change colorscale


            //probabilistic Hough
            std::vector<cv::Vec4i> lines;
            HoughLinesP(dst, lines, 1, (CV_PI / 180), 30, 20, 10);

            std::vector<float> angles;
            std::vector<float>::iterator it;

            std::vector<float> possibleLines;
            std::vector<float>::iterator it2;

            int counter = 0;
            int counter2 = 0;

            for (size_t i = 0; i < lines.size(); i++) {
                cv::Vec4i l = lines[i];
                cv::Point p1 = cv::Point(l[0], l[1]);
                cv::Point p2 = cv::Point(l[2], l[3]);

                float x1 = (float) l[0], y1 = (float) l[1], x2 = (float) l[2], y2 = (float) l[3];
                line(cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, CV_AA);

                if (x2!= x1){
                    float gradient = (y2 - y1) / (x2 - x1);

                    float current_angle = atan(gradient);
                    angles.push_back(current_angle);
                }

            }

            for (size_t i = 0; i < lines.size(); i++){

                cv::Vec4i l = lines[i];
                cv::Point p1 = cv::Point(l[0], l[1]);
                cv::Point p2 = cv::Point(l[2], l[3]);

                float x1 = (float) l[0], y1 = (float) l[1], x2 = (float) l[2], y2 = (float) l[3];

                if (x2-x1 <=20 && y2-y1<=20) { //compare short lines
                    float gradient = (y2 - y1) / (x2 - x1);

                    float current_angle = atan(gradient);

                    //compare with previously stored angles
                    int count = 0;
                    for (it = angles.begin(); it != angles.end(); ++it) {
                        float angle = *it;
                        //take the absolute of difference

                        float difference = abs(current_angle - angle);

                        //if not acute angle subtract value from pi radians
                        if (difference > (M_PI_2)) {
                            difference = M_PI - difference;
                        }


                        if (difference >= (((float) 25 / (float) 180) * M_PI) &&
                            difference <= (((float) 30 / (float) 180) * M_PI)) {
                            count++;

                            if (count>=4) {

                                possibleLines.push_back(angle);

                                line(cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 255, 0), 3,
                                     CV_AA);
                                //counter++;
                            }

                        }


                    }
                }


            }


            //now check if at least 2 lines should be almost parallel among the possibleLines
            for(it2 = possibleLines.begin(); it2!=possibleLines.end(); ++it2){
                float angle = *it2;
                for(std::vector<float>::iterator it3 = possibleLines.begin(); it3!=possibleLines.end(); ++it3){
                    float compare = *it3;
                    if (&compare != &angle){
                        float diff = abs(angle-compare);
                        if (diff > M_PI_2){
                            diff = M_PI - diff;
                        }
                        if (diff<=0.1){
                            counter++;
                        }
                    }
                }
            }

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


            imshow("detected lines", cdst);

            cv::waitKey(1);

        }
    }

    void ImageProcessingNodelet::onInit() {
        img_proc_.reset(new ImageProcessing(getNodeHandle(), getPrivateNodeHandle(), true));;
    }

} // namespace drive_ros_image_recognition

PLUGINLIB_EXPORT_CLASS(drive_ros_image_recognition::ImageProcessingNodelet, nodelet::Nodelet)

