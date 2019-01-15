#ifndef LINE_DETECTION_H
#define LINE_DETECTION_H

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
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
#include "road_model.h"

namespace drive_ros_image_recognition {

class LineDetection {
private:
  // our node handles
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // configs
  float laneWidthWorld_;
  float lineAngle_; // not used at the moment
  float lineVar_;
  float maxSenseRange_;
  int cannyThreshold_;
  int houghThresold_;
  int houghMinLineLen_;
  int houghMaxLineGap_;
  float segmentLength_;
  size_t maxRansacInterations_;
  float trajectoryDist;
  float targetVelocity;

  // variables
  RoadModel roadModel;
  ImageOperator image_operator_;
  int imgHeight_;
  int imgWidth_;
  ros::Time imgTimestamp;
#ifdef PUBLISH_DEBUG
  cv::Mat debugImg_;
#endif

  // communication
  image_transport::ImageTransport imageTransport_;
  image_transport::Subscriber imageSubscriber_;
  ros::Publisher line_output_pub_;
  ros::Publisher trajectoryPub;
  ros::Subscriber homography_params_sub_;
  ros::Subscriber odometrySub;
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
  nav_msgs::Odometry lastUsedOdometry;
  nav_msgs::Odometry latestOdometry;
  bool odometryInitialized = false;
  bool firstUpdate = true;
  ros::Time lastUpdate;

  cv::Mat prevPolyCoeff;

  // callbacks
  void imageCallback(const sensor_msgs::ImageConstPtr& imageIn);
  void odometryCallback(const nav_msgs::OdometryConstPtr &odomMsg);
  void reconfigureCB(drive_ros_image_recognition::LineDetectionConfig& config, uint32_t level);
  dynamic_reconfigure::Server<drive_ros_image_recognition::LineDetectionConfig> dsrv_server_;
  dynamic_reconfigure::Server<drive_ros_image_recognition::LineDetectionConfig>::CallbackType dsrv_cb_;

  // methods
  void findLinesWithHough(cv::Mat &img, std::vector<Line> &houghLines);
  std::vector<tf::Stamped<tf::Point>> findLaneMarkings(std::vector<Line> &lines);

  // helper functions
  float getDistanceBetweenPoints(const cv::Point2f a, const cv::Point2f b) {
    auto dX = a.x - b.x;
    auto dY = a.y - b.y;
    return sqrt(dX * dX + dY * dY);
  }

  std::vector<cv::RotatedRect> buildRegions(cv::Point2f position, float angle);
  void assignLinesToRegions(std::vector<cv::RotatedRect> *regions, std::vector<Line*> &lines,
                            std::vector<Line*> &leftMarkings, std::vector<Line*> &midMarkings,
							std::vector<Line*> &rightMarkings, std::vector<Line*> &otherMarkings);
  bool lineIsInRegion(Line *line, const cv::RotatedRect *region, bool isImageCoordiante) const;
  bool pointIsInRegion(cv::Point2f *pt, cv::Point2f *edges) const;
  Segment findLaneWithRansac(std::vector<Line*> &leftMarkings, std::vector<Line*> &midMarkings, std::vector<Line*> &rightMarkings, cv::Point2f pos, float prevAngle);
  bool findIntersection(Segment &resultingSegment, float segmentAngle, cv::Point2f segStartWorld,
  		std::vector<Line*> &leftMarkings, std::vector<Line*> &midMarkings, std::vector<Line*> &rightMarkings);
  cv::Point2f findTrajectoryPoint(std::vector<tf::Stamped<tf::Point>> &drivingLine);

  bool streetMapInit = false;
  bool resetStreetMap = true;
  cv::Mat binaryStreetMap;
  Line* closestLineToPoint(std::vector<Line> &lines, const cv::Point2f point);
  float pointToLineDistance(Line &l, const cv::Point2f &p);
  float distanceBetweenLines(Line &a, Line &b);
  void createBinaryStreetMap(std::vector<Line> &lines, ros::Time stamp);


#ifdef PUBLISH_DEBUG
  void drawDebugLines(std::vector<Line> &lines);
#endif

public:
  LineDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh);
  ~LineDetection();
  bool init();
};

} // namepsace drive_ros_image_recognition

#endif // LINE_DETECTION_H
