#include <drive_ros_image_recognition/new_road_detection.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "new_road_detection");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  NewRoadDetection new_road_detection(nh,pnh);
  if (!new_road_detection.init()) {
    return 1;
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
