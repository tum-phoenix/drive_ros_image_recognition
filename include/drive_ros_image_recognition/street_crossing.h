#ifndef STREET_CROSSING_H
#define STREET_CROSSING_H

#include <drive_ros_image_recognition/detection.h>
#include <drive_ros_image_recognition/CrossingDetectionConfig.h>

namespace drive_ros_image_recognition {

class StreetCrossingDetection : public Detection<drive_ros_image_recognition::CrossingDetectionConfig> {
protected:
    virtual void imageCallback(const sensor_msgs::ImageConstPtr& img_in);
    virtual bool find();
public:
    StreetCrossingDetection();
    StreetCrossingDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh, image_transport::ImageTransport* it);
    ~StreetCrossingDetection();
};

} // namepsace drive_ros_image_recognition

#endif // STREET_CROSSING_H
