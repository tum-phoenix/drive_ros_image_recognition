#ifndef WARP_IMAGE_H
#define WARP_IMAGE_H

#include <ros/ros.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

namespace drive_ros_image_recognition {

class WarpContent {
public:
  WarpContent(ros::NodeHandle& pnh);
  ~WarpContent();
  bool init();
private:
  void world_image_callback(const sensor_msgs::ImageConstPtr& msg);
  ros::NodeHandle pnh_;
  cv::Mat current_image_;
  // needs two components: camera model and homography for transformation
  // homography matrices for transformation
  cv::Mat world2cam_;
  cv::Mat cam2world_;
  // camera model matrix and distortion coefficients
  cv::Mat cam_mat_;
  cv::Mat dist_coeffs_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_sub_;
  image_transport::Publisher img_pub_;
};

} // namespace drive_ros_image_recognition

#endif
