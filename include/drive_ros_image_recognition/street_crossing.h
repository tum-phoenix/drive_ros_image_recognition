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

#include "drive_ros_image_recognition/RoadLane.h"

namespace drive_ros_image_recognition {

class StreetCrossingDetection {

private:
    // configs
    int sobelThreshold_;
    float minLineWidthMul_;
    float maxLineWidthMul_;
    float lineWidth_;

    // variables
    std::vector<SearchLine> searchLines_;
    CvImagePtr currentImage_;

    // communication
    image_transport::ImageTransport imageTransport_;
    image_transport::Subscriber imageSubscriber_;

    ros::Publisher line_output_pub_;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    void imageCallback(const sensor_msgs::ImageConstPtr& imageIn);


#ifdef DRAW_DEBUG
    image_transport::Publisher debugImagePublisher;
    cv::Mat debugImage;
#endif

    // methods
    bool findStartline();

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
