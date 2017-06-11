#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher_node");
  ros::NodeHandle nh("~");
  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("img_out", 1);

  cv_bridge::CvImage cv_image;
  sensor_msgs::Image ros_image;

  double publish_rate_received = 0.1;
  if (!nh.getParam("publish_rate",publish_rate_received))
    ROS_WARN_STREAM("Unable to get 'publish_rate' parameter, using default: "<<publish_rate_received);
  ros::Rate publish_rate(publish_rate_received);

  std::string image_folder;
  if (!nh.getParam("image_folder",image_folder)) {
    ROS_ERROR("Unable to get 'image_folder' parameter, shutting down node.");
    return 1;
  }

  std::vector<cv::String> filenames;
  cv::String folder = image_folder;
  cv::glob(folder, filenames);

  if (filenames.size() == 0) {
    ROS_ERROR("No images found in provided folder, shutting down");
    return 1;
  }

  size_t i = 0;
  while (ros::ok() && i<filenames.size()) {
      cv_image.image = cv::imread(filenames[i], CV_LOAD_IMAGE_ANYDEPTH);

      if(!cv_image.image.data) {
        ROS_WARN_STREAM("Problem loading image "<<filenames[i]<<", skipping");
        continue;
      }

      cv_image.toImageMsg(ros_image);
      pub.publish(ros_image);
      // this is non-interruptible, so keep the rate low, or node will not shut down cleanly
      publish_rate.sleep();
      ++i;
  }
  return 0;
}
