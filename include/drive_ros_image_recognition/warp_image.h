#ifndef WARP_IMAGE_Hag
#define WARP_IMAGE_H

#include <ros/ros.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <drive_ros_image_recognition/ImageToWorld.h>
#include <drive_ros_image_recognition/WorldToImage.h>
#include <tf/transform_listener.h>

#include <nodelet/nodelet.h>

namespace drive_ros_image_recognition {

class WarpContent {
public:
  WarpContent(const ros::NodeHandle& pnh);
  ~WarpContent();
  bool init();
private:
  void world_image_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
  bool worldToImage(drive_ros_image_recognition::WorldToImage::Request &req,
                    drive_ros_image_recognition::WorldToImage::Response &res);
  bool imageToWorld(drive_ros_image_recognition::ImageToWorld::Request  &req,
                    drive_ros_image_recognition::ImageToWorld::Response &res);
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
  image_transport::CameraSubscriber cam_sub_;
  image_transport::Publisher img_pub_;
  // debug undistort publisher
  image_transport::Publisher undistort_pub_;
  ros::ServiceServer worldToImageServer_;
  ros::ServiceServer imageToWorldServer_;
  // todo: moving to camera model subscription
  image_geometry::PinholeCameraModel cam_model_;
  tf::TransformListener tf_listener_;
};

class WarpImageNodelet : public nodelet::Nodelet {
public:
    virtual void onInit();
private:
    std::unique_ptr<WarpContent> my_content_;
};

} // namespace drive_ros_image_recognition

#endif
