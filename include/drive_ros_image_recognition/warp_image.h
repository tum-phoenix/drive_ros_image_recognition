#ifndef WARP_IMAGE_H
#define WARP_IMAGE_H

#include <ros/ros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <drive_ros_image_recognition/common_image_operations.h>

namespace drive_ros_image_recognition {

class WarpContent {
public:
  WarpContent(const ros::NodeHandle &nh, const ros::NodeHandle& pnh);
  ~WarpContent();
  bool init();
private:
  void world_image_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;
  cv::Mat current_image_;
  ImageOperator image_operator_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber cam_sub_;
  image_transport::Publisher img_pub_;
  image_transport::Publisher undistort_pub_;
  // todo: moving to camera model subscription
  image_geometry::PinholeCameraModel cam_model_;
};

class WarpImageNodelet : public nodelet::Nodelet {
public:
    virtual void onInit();
private:
    std::unique_ptr<WarpContent> my_content_;
};

} // namespace drive_ros_image_recognition

#endif
