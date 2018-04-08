#include <drive_ros_image_recognition/crosswalk_detection.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "crosswalk_detection");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

#ifndef NDEBUG
  // give GDB time to attach
  ros::Duration(2.0).sleep();
#endif

  drive_ros_image_recognition::CrosswalkDetection crosswalk_detection(nh,pnh);
  if (!crosswalk_detection.init()) {
    return 1;
  }
  else {
    ROS_INFO("crosswalk detection node succesfully initialized");
  }

  while (ros::ok()) {
    ros::spin();
  }
  return 0;
}
