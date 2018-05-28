#ifndef LINE_DETECTION_H
#define LINE_DETECTION_H

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <drive_ros_image_recognition/common_image_operations.h>
#include <drive_ros_image_recognition/LineDetectionConfig.h>
#include <nodelet/nodelet.h>
#include <tf/transform_listener.h>
#include "drive_ros_msgs/RoadLane.h"
#include "line.h"

namespace drive_ros_image_recognition {

class LineDetection {
private:
  // our node handles
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // configs
  float lineWidth_;
  float lineAngle_;
  float lineVar_;
  float maxViewRange_;
  int cannyThreshold_;
  int houghThresold_;
  int houghMinLineLen_;
  int houghMaxLineGap_;

  // variables
  CvImagePtr currentImage_;
  std::vector<Line> currentGuess_;
  std::vector<Line> currentMiddleLine_;
  ImageOperator image_operator_;

  // communication
  image_transport::ImageTransport imageTransport_;
  image_transport::Subscriber imageSubscriber_;
  ros::Publisher line_output_pub_;
  ros::Subscriber homography_params_sub_;
#ifdef PUBLISH_DEBUG
  image_transport::Publisher debugImgPub_;
#endif

  // homography components
  cv::Mat world2cam_;
  cv::Mat cam2world_;
  cv::Mat scaling_mat_;
  cv::Mat scaling_mat_inv_;
  cv::Size transformed_size_;
  bool homog_received_;

  // odometry components
  tf::TransformListener tfListener_;
  cv::Point2f oldPoint_;

  // callbacks
  void imageCallback(const sensor_msgs::ImageConstPtr& imageIn);
  void reconfigureCB(drive_ros_image_recognition::LineDetectionConfig& config, uint32_t level);
  dynamic_reconfigure::Server<drive_ros_image_recognition::LineDetectionConfig> dsrv_server_;
  dynamic_reconfigure::Server<drive_ros_image_recognition::LineDetectionConfig>::CallbackType dsrv_cb_;

  // methods
  void findLane();

  float getDistanceBetweenPoints(const cv::Point2f a, const cv::Point2f b) {
    auto dX = a.x - b.x;
    auto dY = a.y - b.y;
    return sqrt(dX * dX + dY * dY);
  }

public:
  LineDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh);
  ~LineDetection();
  bool init();
};

} // namepsace drive_ros_image_recognition

#endif // LINE_DETECTION_H
