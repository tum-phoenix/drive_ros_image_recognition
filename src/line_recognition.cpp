#include <drive_ros_image_recognition/line_recognition.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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

  if( !cv_ptr->image.data)
  {
    ROS_WARN("Empty image received, skipping!");
    return;
  }

  // define some processing constants
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;
  cv::Mat grad;

  cv::Mat img_blurred;
  cv::GaussianBlur( cv_ptr->image, img_blurred, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );

  // convert to grayscale
  cv::cvtColor( img_blurred, img_blurred, CV_BGR2GRAY );

  /// Create window
  cv::namedWindow( "Sobel", CV_WINDOW_AUTOSIZE );

  /// Generate grad_x and grad_y
  cv::Mat grad_x, grad_y;
  cv::Mat abs_grad_x, abs_grad_y;

  /// Gradient X
  cv::Sobel( img_blurred, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
  cv::convertScaleAbs( grad_x, abs_grad_x );

  /// Gradient Y
  cv::Sobel( img_blurred, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
  cv::convertScaleAbs( grad_y, abs_grad_y );

  /// Total Gradient (approximate)
  cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

  cv::imshow( "Sobel", grad );

  cv::waitKey(0);
}

} // end drive_ros_image_recognition namespace
