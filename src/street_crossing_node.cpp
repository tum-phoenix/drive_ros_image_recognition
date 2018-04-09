#include <drive_ros_image_recognition/street_crossing.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "street_crossing_detection");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  image_transport::ImageTransport* it = new image_transport::ImageTransport(pnh);
  drive_ros_image_recognition::StreetCrossingDetection streetCrossingDetection(nh, pnh, it);
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
