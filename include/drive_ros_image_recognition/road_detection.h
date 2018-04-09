#ifndef ROAD_DETECTION_H
#define ROAD_DETECTION_H

#include <drive_ros_image_recognition/detection.h>
#include <drive_ros_image_recognition/RoadDetectionConfig.h>

// for multithreading
#include <mutex>
#include <condition_variable>
#include <thread>

namespace drive_ros_image_recognition {

/**
 * @brief Port of LMS module road_detection to ROS
 **/
class RoadDetection: public Detection<RoadDetectionConfig> {
    // testing of hint adjustement
    tf::StampedTransform last_received_transform_;
    tf::TransformListener tf_listener_;

    std::list<SearchLine> lines_;
    ros::Publisher line_output_pub_;
    std::vector<geometry_msgs::PointStamped> road_points_buffer_;
#ifdef PUBLISH_WORLD_POINTS
    ros::Publisher world_point_pub_;
#endif

    void debugImageCallback(const sensor_msgs::ImageConstPtr& img_in);
    void debugDrawFrameCallback(const sensor_msgs::ImageConstPtr& img_in,
                                const std::string camera_frame,
                                const std::string draw_frame);
    geometry_msgs::PointStamped processSearchLine(const SearchLine &line);

    virtual void imageCallback(const sensor_msgs::ImageConstPtr& img_in);
    virtual void syncCallback(const sensor_msgs::ImageConstPtr& img_in, const drive_ros_msgs::RoadLaneConstPtr& road_in);
    virtual bool find();
public:
    RoadDetection();
    RoadDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh, image_transport::ImageTransport* it);
    ~RoadDetection();
    bool init();
};

}
#endif // ROAD_DETECTION_H
