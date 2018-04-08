#ifndef CROSSWALK_DETECTION_H
#define CROSSWALK_DETECTION_H

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <drive_ros_msgs/RoadLane.h>
#include <dynamic_reconfigure/server.h>
#include <drive_ros_image_recognition/geometry_common.h>
#include <drive_ros_image_recognition/common_image_operations.h>
#include <nodelet/nodelet.h>
#include <list>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/Image.h>
#include <drive_ros_msgs/RoadLane.h>

namespace drive_ros_image_recognition {

/**
 * @brief Simple crosswalk detector based on the amount of white stripes perpendicular to the road
 **/
class CrosswalkDetection {
    // configs
    float searchOffset_;
    float distanceBetweenSearchlines_;
    bool findPointsBySobel_;
    bool translateEnvironment_;
    int sobelThreshold_;
    double laneWidth_;

    // temporary testing of hint adjustement
    tf::StampedTransform last_received_transform_;
    tf::TransformListener tf_listener_;


    // subscribers and publishers
    image_transport::ImageTransport it_;
    // image subscriber, handles hints internally, used for debugging
    image_transport::Subscriber img_sub_standalone_;
//    std::unique_ptr<image_transport::SubscriberFilter> img_sub_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image> > img_sub_;
    std::unique_ptr<message_filters::Subscriber<drive_ros_msgs::RoadLane> > road_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, drive_ros_msgs::RoadLane> SyncImageToRoad;
    std::unique_ptr<message_filters::Synchronizer<SyncImageToRoad> > sync_;
//    std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, drive_ros_msgs::RoadLane> > sync_;
#ifdef PUBLISH_DEBUG
    image_transport::Publisher visualization_pub_;
#endif

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    CvImagePtr current_image_;
    std::vector<geometry_msgs::PointStamped> road_hints_buffer_;
    ImageOperator image_operator_;

    void imageCallback(const sensor_msgs::ImageConstPtr& img_in);
    void syncCallback(const sensor_msgs::ImageConstPtr& img_in, const drive_ros_msgs::RoadLaneConstPtr& road_in);
    bool find();
#ifdef PUBLISH_WORLD_POINTS
    ros::Publisher world_point_pub_;
#endif

public:
    CrosswalkDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh);
    ~CrosswalkDetection();
    bool init();
};

class CrosswalkDetectionNodelet: public nodelet::Nodelet {
public:
  virtual void onInit();
private:
  std::unique_ptr<CrosswalkDetection> crosswalk_detection_;
};

}
#endif // CROSSWALK_DETECTION_H
