#ifndef WARP_IMAGE_H
#define WARP_IMAGE_H

#include <ros/ros.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <drive_ros_image_recognition/ImageToWorld.h>
#include <drive_ros_image_recognition/WorldToImage.h>

#include <nodelet/nodelet.h>

namespace drive_ros_image_recognition {

class WarpContent {
public:
  WarpContent(const ros::NodeHandle& pnh);
  ~WarpContent();
  bool init();
private:
  void world_image_callback(const sensor_msgs::ImageConstPtr& msg);
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
  image_transport::Subscriber img_sub_;
  image_transport::Publisher img_pub_;
  // debug undistort publisher
  image_transport::Publisher undistort_pub_;
  ros::ServiceServer worldToImageServer_;
  ros::ServiceServer imageToWorldServer_;
  // todo: move to using this in the future
//  sensor_msgs::CameraInfo cam_info_;
};

class WarpImageNodelet : public nodelet::Nodelet {
public:
    virtual void onInit();
private:
    std::unique_ptr<WarpContent> my_content_;
};

} // namespace drive_ros_image_recognition

#endif
