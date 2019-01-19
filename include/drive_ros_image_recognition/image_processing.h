#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <opencv2/opencv.hpp>

namespace drive_ros_image_recognition {

class ImageProcessing
{
public:
  ImageProcessing(ros::NodeHandle nh, ros::NodeHandle pnh, bool nodelet=false);
  bool detectLaneLines(cv::Mat image, cv::Vec4i& l1, cv::Vec4i& l2);
  void extractRelevantRegion(cv::Mat img_in, cv::Mat& img_out, const cv::Vec4i l1, const cv::Vec4i l2);
private:
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  image_transport::Subscriber img_sub_;
  bool nodelet_;
};

class ImageProcessingNodelet : public nodelet::Nodelet {
public:
    virtual void onInit();
private:
    std::unique_ptr<ImageProcessing> img_proc_;
};

} // namespace drive_ros_image_recognition

#endif //IMAGE_PROCESSING_H
