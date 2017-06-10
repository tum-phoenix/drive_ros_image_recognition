#include <drive_ros_image_recognition/line_recognition.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace drive_ros_image_recognition {

LineRecognition::LineRecognition( ros::NodeHandle nh ):
  it_(nh)
{
  img_sub_ = it_.subscribe("img_in", 10, &LineRecognition::img_callback, this);
}

LineRecognition::~LineRecognition() {
}

void LineRecognition::img_callback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // process received image
}

} // end drive_ros_image_recognition namespace
