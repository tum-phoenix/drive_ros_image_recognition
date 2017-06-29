#include <drive_ros_image_recognition/street_crossing.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "street_crossing_detection");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  drive_ros_image_recognition::detection::StreetCrossingDetection streetCrossingDetection(nh, pnh);
  if (!streetCrossingDetection.init()) {
    return 1;
  }
  else {
    ROS_INFO("Street crossing detection node succesfully initialized");
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
