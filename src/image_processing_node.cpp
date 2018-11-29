#include <drive_ros_image_recognition/image_processing.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_processing_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  drive_ros_image_recognition::ImageProcessing img_proc(nh, pnh);
#ifndef NDEBUG
  // give GDB time to attach
  ros::Duration(2.0).sleep();
#endif

  while (ros::ok())
    ros::spin();
  return 0;
}
