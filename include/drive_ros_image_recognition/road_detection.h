#ifndef ROAD_DETECTION_H
#define ROAD_DETECTION_H

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <drive_ros_image_recognition/RoadLane.h>
#include <dynamic_reconfigure/server.h>
#include <drive_ros_image_recognition/LineDetectionConfig.h>
#include <drive_ros_image_recognition/geometry_common.h>
#include <drive_ros_image_recognition/common_image_operations.h>
#include <nodelet/nodelet.h>
#include <list>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

// for multithreading
#include <mutex>
#include <condition_variable>
#include <thread>

namespace drive_ros_image_recognition {

/**
 * @brief Port of LMS module road_detection to ROS
 **/
class RoadDetection {
    // configs
    float searchOffset_;
    float distanceBetweenSearchlines_;
    bool findPointsBySobel_;
    float minLineWidthMul_;
    float maxLineWidthMul_;
    int brightness_threshold_;
    float laneWidthOffsetInMeter_;
    bool translateEnvironment_;
    bool useWeights_;
    int sobelThreshold_;

    // ported in the WarpImage class, ports the entire image already
//    lms::imaging::Homography homo;
    //Datachannels
    // todo: we have not determined all interfaces yet, so will leave this in for now until we have figured it out
//    lms::ReadDataChannel<lms::imaging::Image> image;
//    lms::WriteDataChannel<street_environment::RoadLane> road;
//    lms::WriteDataChannel<street_environment::RoadLane> output;
//    lms::WriteDataChannel<lms::imaging::Image> debugImage;
//    lms::WriteDataChannel<lms::math::polyLine2f> debugAllPoints;
//    lms::WriteDataChannel<lms::math::polyLine2f> debugValidPoints;
//    lms::WriteDataChannel<lms::math::polyLine2f> debugTranslatedPoints;
//    lms::ReadDataChannel<street_environment::CarCommand> car;

    std::list<SearchLine> lines_;

    int linesToProcess_;
    CvImagePtr current_image_;
    image_transport::ImageTransport it_;
    // road inputs and outputs
    image_transport::Subscriber img_sub_debug_;
//    std::unique_ptr<image_transport::SubscriberFilter> img_sub_;
//    std::unique_ptr<message_filters::Subscriber<RoadLane> > road_sub_;
//    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, RoadLane> SyncImageToRoad;
//    std::unique_ptr<message_filters::Synchronizer<SyncImageToRoad> > sync_;
    ros::Publisher line_output_pub_;
#ifdef PUBLISH_DEBUG
    image_transport::Publisher debug_img_pub_;
    image_transport::Publisher detected_points_pub_;
    image_transport::Publisher filtered_points_pub_;
#endif
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::vector<geometry_msgs::Point32> road_points_buffer_;

    dynamic_reconfigure::Server<drive_ros_image_recognition::LineDetectionConfig> dsrv_server_;
    dynamic_reconfigure::Server<drive_ros_image_recognition::LineDetectionConfig>::CallbackType dsrv_cb_;
    void debugImageCallback(const sensor_msgs::ImageConstPtr& img_in);
    void syncCallback(const sensor_msgs::ImageConstPtr& img_in, const RoadLaneConstPtr& road_in);
    void reconfigureCB(drive_ros_image_recognition::LineDetectionConfig& config, uint32_t level);
    bool find();
    void processSearchLine(const SearchLine &line);
    TransformHelper transform_helper_;
    ImageOperator image_operator_;

public:
    RoadDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh);
    ~RoadDetection();
    bool init();
};

class RoadDetectionNodelet: public nodelet::Nodelet {
public:
  virtual void onInit();
private:
  std::unique_ptr<RoadDetection> road_detection_;
};

}
#endif // ROAD_DETECTION_H
