#include <ros/ros.h>
#include <drive_ros_image_recognition/line_recognition.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "line_recognition");
  ros::NodeHandle nh;

  drive_ros_image_recognition::LineRecognition line_rec = drive_ros_image_recognition::LineRecognition(nh);

#ifndef NDEBUG
  // give GDB time to attach
  ros::Duration(1.0).sleep();
#endif

  while (ros::ok()) {
    ros::spin();
  }
  return 0;
}
