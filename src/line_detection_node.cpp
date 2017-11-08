#include <drive_ros_image_recognition/line_detection.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "line_detection");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  drive_ros_image_recognition::LineDetection lineDetection(nh, pnh);
  if (!lineDetection.init()) {
    return 1;
  }
  else {
    ROS_INFO("Line detection node succesfully initialized");
  }

#ifndef NDEBUG
  // give GDB time to attach
  ros::Duration(1.0).sleep();
#endif

  while (ros::ok()) {
    ros::spin();
  }
  return 0;
}
