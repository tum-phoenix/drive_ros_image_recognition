#ifndef STREET_CROSSING_H
#define STREET_CROSSING_H

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
#include "drive_ros_msgs/RoadLane.h"

namespace drive_ros_image_recognition {

// todo: move to own file

class StreetLine {
public:
  cv::Point p1_, p2_, mid_;
  int vote;

  StreetLine(cv::Point p1, cv::Point p2)
    : vote(0)
  {
    if(p1.y < p2.y) {
      p1_ = p1;
      p2_ = p2;
    } else {
      p1_ = p2;
      p2_ = p1;
    }
    mid_ = (p1 + p2) * .5;
  }

  StreetLine(cv::Vec4i c)
    : vote(0)
  {
    cv::Point p1(c[0], c[1]);
    cv::Point p2(c[2], c[3]);
    if(p1.y < p2.y) {
      p1_ = p1;
      p2_ = p2;
    } else {
      p1_ = p2;
      p2_ = p1;
    }
    mid_ = ((p1_ + p2_) * .5);
  }

  // returns the angle of the lines between 0 and PI
  inline double getAngle() { return abs(atan2(p1_.y - p2_.y, p1_.x - p2_.x)); }

  // used to sort lines ascending wrt y-coordinate
  bool operator() (StreetLine sl1, StreetLine sl2) { return sl1.mid_.y < sl2.mid_.y; }
};

class StreetCrossingDetection {

private:
    // configs
    int sobelThreshold_;
    float minLineWidthMul_;
    float maxLineWidthMul_;
    float lineWidth_;

    int cannyThreshold_;
    int houghThresold_;
    int houghMinLineLen_;
    int houghMaxLineGap_;
    int sameLineThreshold_;

    // variables
    std::vector<SearchLine> searchLines_;
    CvImagePtr currentImage_;

//    cv::Mat warpedImg_;
//    std::vector<StreetLine> streetLines_;

    int startLineCounter_;
    cv::Point lastStartLinePos_;

    // communication
    image_transport::ImageTransport imageTransport_;
    image_transport::Subscriber imageSubscriber_;
    image_transport::Subscriber warpedImageSubscriber_;

    ros::Publisher line_output_pub_;
#ifdef PUBLISH_DEBUG
    image_transport::Publisher debugImagePublisher_;
#endif

    // homography components
      ros::Subscriber homography_params_sub_;
      cv::Mat world2cam_;
      cv::Mat cam2world_;
      cv::Mat scaling_mat_;
      cv::Mat scaling_mat_inv_;
      cv::Size transformed_size_;
      bool homo_received_;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    void imageCallback(const sensor_msgs::ImageConstPtr& imageIn);
    void warpedImageCallback(const sensor_msgs::ImageConstPtr& warpedImageIn);

    // methods
    bool findStartline();
    void findLane();

    // this vector must always be orderer by HoughLine.mid_ (ascending)
    std::vector<StreetLine> currentLaneMid;
    bool onRightLine;

    TransformHelper transform_helper_;
    ImageOperator image_operator_;

    void reconfigureCB(drive_ros_image_recognition::LineDetectionConfig& config, uint32_t level);
    dynamic_reconfigure::Server<drive_ros_image_recognition::LineDetectionConfig> dsrv_server_;
    dynamic_reconfigure::Server<drive_ros_image_recognition::LineDetectionConfig>::CallbackType dsrv_cb_;

public:
    StreetCrossingDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh);
    ~StreetCrossingDetection();
    bool init();
};

class StreetCrossingDetectionNodelet: public nodelet::Nodelet {
public:
  virtual void onInit();
private:
  std::unique_ptr<StreetCrossingDetection> street_crossing_detection_;
};


} // namepsace drive_ros_image_recognition

#endif // STREET_CROSSING_H
