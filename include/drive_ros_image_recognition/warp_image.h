#ifndef WARP_IMAGE_H
#define WARP_IMAGE_H

#include <ros/ros.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <drive_ros_image_recognition/common_image_operations.h>

#include <nodelet/nodelet.h>

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
  // needs two components: camera model and homography for transformation
  // camera model matrix and distortion coefficients
  cv::Mat cam_mat_;
  cv::Mat dist_coeffs_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber cam_sub_;
  image_transport::Publisher img_pub_;
  // debug undistort publisher
  image_transport::Publisher undistort_pub_;
  ros::ServiceServer worldToImageServer_;
  ros::ServiceServer imageToWorldServer_;
  // todo: moving to camera model subscription
  image_geometry::PinholeCameraModel cam_model_;

  // homography components
  ros::Subscriber homography_params_sub_;
  cv::Mat world2cam_;
  cv::Mat cam2world_;
  cv::Mat scaling_mat_;
  cv::Mat scaling_mat_inv_;
  cv::Mat scaledCam2world_;
  cv::Mat scaledWorld2cam_;
  cv::Size transformed_size_;
  bool homo_received_;
};

class WarpImageNodelet : public nodelet::Nodelet {
public:
    virtual void onInit();
private:
    std::unique_ptr<WarpContent> my_content_;
};

} // namespace drive_ros_image_recognition

#endif
