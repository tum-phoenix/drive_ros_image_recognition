#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <drive_ros_image_recognition/BarredAreaDetectionConfig.h>

namespace drive_ros_image_recognition {

class ImageProcessing
{
public:
  ImageProcessing(ros::NodeHandle nh, ros::NodeHandle pnh, bool nodelet=false);
private:
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  void reconfigureCallback(drive_ros_image_recognition::BarredAreaDetectionConfig &config, uint32_t level);
  image_transport::Subscriber img_sub_;
  float squared_length_threshold_ = 400.f;
  bool nodelet_;
  dynamic_reconfigure::Server<drive_ros_image_recognition::BarredAreaDetectionConfig> reconfigure_server_;
  int canny_param1_ = 20;
  int canny_param2_ = 150;
  int hough_threshold_ = 30;
  int hough_min_line_length_ = 20;
  int hough_max_line_gap_ = 10;
};

class ImageProcessingNodelet : public nodelet::Nodelet {
public:
    virtual void onInit();
private:
    std::unique_ptr<ImageProcessing> img_proc_;
};

} // namespace drive_ros_image_recognition

#endif //IMAGE_PROCESSING_H
