#include <ros/ros.h>
#include <drive_ros_image_recognition/warp_image.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "warp_image_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  drive_ros_image_recognition::WarpContent warp(nh,pnh);
  if (!warp.init()) {
    return 1;
  }
  else {
    ROS_INFO("Warp_image node succesfully initialized");
  }

#ifndef NDEBUG
  // give GDB time to attach
  ros::Duration(2.0).sleep();
#endif

  while (ros::ok()) {
    ros::spin();
  }
  return 0;
}
