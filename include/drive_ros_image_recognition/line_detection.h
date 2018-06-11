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

  // drive state enum
  enum DriveState {
      StartBox = 0,
      Parking = 1,
      Intersection = 2,
      Street = 3
  };

  // drive mode config
  bool isObstaceCourse; // true for Obstacle Evasion Course; false for Free Drive (w/o Obstacles) & Parking
  DriveState driveState;

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
//  CvImagePtr currentImage_;
  std::vector<Line> currentGuess_;
  std::vector<Line> currentMiddleLine_;
  ImageOperator image_operator_;
  int imgHeight_;
#ifdef PUBLISH_DEBUG
  cv::Mat debugImg_;
#endif

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
  void findLinesWithHough(CvImagePtr img, std::vector<Line> &houghLines);
  bool findLanesFromStartbox(std::vector<Line> &lines); // called at the start to initialy find the lanes
  bool findLaneSimple(std::vector<Line> &lines); // finds the lane assuming there are no markings missing (e.g. for Parking area)
  bool findLaneAdvanced(std::vector<Line> &lines); // finds the lane assuming some markings could be missing
  bool findIntersectionExit(std::vector<Line> &lines); // find the end of an intersection (used while in intersection)

  // helper functions
  float getDistanceBetweenPoints(const cv::Point2f a, const cv::Point2f b) {
    auto dX = a.x - b.x;
    auto dY = a.y - b.y;
    return sqrt(dX * dX + dY * dY);
  }

  void buildBbAroundLines(std::vector<cv::Point2f> &centerPoints, std::vector<cv::Point2f> &midLinePoints);

public:
  LineDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh);
  ~LineDetection();
  bool init();
};

} // namepsace drive_ros_image_recognition

#endif // LINE_DETECTION_H
