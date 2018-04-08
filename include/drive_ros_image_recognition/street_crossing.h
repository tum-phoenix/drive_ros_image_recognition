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
#include <drive_ros_image_recognition/CrossingDetectionConfig.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/Image.h>

namespace drive_ros_image_recognition {

class StreetCrossingDetection {

private:
    // variables
    ImageOperator image_operator_;
    CvImagePtr currentImage_;
    // road hints are assumed to be in the center of the currently driven lane
    std::vector<geometry_msgs::PointStamped> road_hints_buffer_;
    drive_ros_image_recognition::CrossingDetectionConfig config_;

    // communication
    image_transport::ImageTransport imageTransport_;
    std::unique_ptr<image_transport::SubscriberFilter> img_sub_;
    std::unique_ptr<message_filters::Subscriber<drive_ros_msgs::RoadLane> > road_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, drive_ros_msgs::RoadLane> SyncImageToHints;
    std::unique_ptr<message_filters::Synchronizer<SyncImageToHints> > sync_;
    image_transport::Subscriber img_sub_standalone;
    void syncCallback(const sensor_msgs::ImageConstPtr& img_in, const drive_ros_msgs::RoadLaneConstPtr& road_in);
    void imageCallback(const sensor_msgs::ImageConstPtr& img_in);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

#ifdef DRAW_DEBUG
    image_transport::Publisher debugImagePublisher;
    cv::Mat debugImage;
#endif

    // methods
    bool find();

    // dynamic reconfigure
    void reconfigureCB(drive_ros_image_recognition::CrossingDetectionConfig& config, uint32_t level);
    dynamic_reconfigure::Server<drive_ros_image_recognition::CrossingDetectionConfig> dsrv_server_;

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
