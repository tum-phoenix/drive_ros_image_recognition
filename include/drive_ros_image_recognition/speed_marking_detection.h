#ifndef SPEED_MARKING_DETECTION_H
#define SPEED_MARKING_DETECTION_H

#include <drive_ros_image_recognition/detection.h>
#include <drive_ros_image_recognition/SpeedMarkingDetectionConfig.h>

namespace drive_ros_image_recognition {

class SpeedMarkingDetection : public Detection<drive_ros_image_recognition::SpeedMarkingDetectionConfig> {
protected:
    std::string NM1_;
    std::string NM2_;
    virtual bool find();
public:
    SpeedMarkingDetection();
    SpeedMarkingDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh, image_transport::ImageTransport* it);
    ~SpeedMarkingDetection();
    bool init();
};

} // namepsace drive_ros_image_recognition

#endif // SPEED_MARKING_DETECTION_H
