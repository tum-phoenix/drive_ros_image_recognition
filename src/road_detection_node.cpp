#include <drive_ros_image_recognition/road_detection.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "road_detection");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

#ifndef NDEBUG
  // give GDB time to attach
  ros::Duration(2.0).sleep();
#endif

  image_transport::ImageTransport* it = new image_transport::ImageTransport(pnh);
  drive_ros_image_recognition::RoadDetection road_detection(nh,pnh,it);
  if (!road_detection.init()) {
    return 1;
  }
  else {
    ROS_INFO("road detection node succesfully initialized");
  }

  while (ros::ok()) {
    ros::spin();
  }
  return 0;
}
