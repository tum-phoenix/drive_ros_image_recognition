#ifndef CROSSWALK_DETECTION_H
#define CROSSWALK_DETECTION_H

#include <drive_ros_image_recognition/detection.h>
#include <drive_ros_image_recognition/CrosswalkDetectionConfig.h>

namespace drive_ros_image_recognition {

/**
 * @brief Simple crosswalk detector based on the amount of white stripes perpendicular to the road
 **/
class CrosswalkDetection : public Detection<drive_ros_image_recognition::CrosswalkDetectionConfig> {
private:
    virtual void imageCallback(const sensor_msgs::ImageConstPtr& img_in);
    virtual bool find();
public:
    CrosswalkDetection();
    CrosswalkDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh, image_transport::ImageTransport* it);
    ~CrosswalkDetection();
};

}
#endif // CROSSWALK_DETECTION_H
