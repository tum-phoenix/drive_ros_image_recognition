#ifndef LINE_RECOGNITION_H
#define LINE_RECOGNTION_H

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

namespace drive_ros_image_recognition {

class LineRecognition {
public:
  LineRecognition(ros::NodeHandle nh);
  ~LineRecognition();

  void img_callback(const sensor_msgs::ImageConstPtr& msg);

private:
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_sub_;
};

} // namespace drive_ros_image_recognition

#endif // LINE_RECOGNTION_H
