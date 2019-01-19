#include <drive_ros_image_recognition/image_processing.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pluginlib/class_list_macros.h>


namespace drive_ros_image_recognition {

ImageProcessing::ImageProcessing(ros::NodeHandle nh, ros::NodeHandle pnh, bool nodelet) : nodelet_(nodelet)
{
  image_transport::ImageTransport it(pnh);
  img_sub_ = it.subscribe("img_in", 1, &ImageProcessing::imageCallback, this);
}

void ImageProcessing::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED("ImageProcessing", "cv_bridge exception: "<<e.what());
    return;
  }

  // example content: display incoming image
  // (NOTE: default OpenCV display functions don't work with nodelets as they are not suited for multi-threaded applications)
  if (!nodelet_)
  {
    cv::Mat img_in = cv_ptr->image;
    cv::namedWindow("Incoming image", CV_WINDOW_NORMAL);
    cv::imshow("Incoming image", img_in);
    cv::waitKey(1);
  }
}

void ImageProcessingNodelet::onInit()
{
  img_proc_.reset(new ImageProcessing(getNodeHandle(), getPrivateNodeHandle(), true));;
}

} // namespace drive_ros_image_recognition

PLUGINLIB_EXPORT_CLASS(drive_ros_image_recognition::ImageProcessingNodelet, nodelet::Nodelet)

