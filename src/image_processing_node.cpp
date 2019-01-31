#include <drive_ros_image_recognition/image_processing.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_processing_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

#ifndef NDEBUG
  // give GDB time to attach
  ros::Duration(2.0).sleep();
#endif

  drive_ros_image_recognition::ImageProcessing img_proc(nh, pnh);

  while (ros::ok())
    ros::spin();
  return 0;
}
