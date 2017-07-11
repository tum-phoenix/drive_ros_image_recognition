#include <drive_ros_image_recognition/line_recognition.h>
#include <iostream>
#include <fstream>
#include <string>
#include <iterator>
#include <opencv2/core/ocl.hpp>

#define USE_UMAT

#ifdef USE_UMAT
  #define MAT cv::UMat
#else
  #define MAT cv::Mat
#endif

using namespace std;

namespace drive_ros_image_recognition {

LineRecognition::LineRecognition( ros::NodeHandle nh ):
  it_(nh)
{
  img_sub_ = it_.subscribe("/camera1/image_raw", 10, &LineRecognition::img_callback, this);

  if (!cv::ocl::haveOpenCL())
  {
      cout << "OpenCL is not avaiable..." << endl;
      return;
  }
  cv::ocl::Context context;
  if (!context.create(cv::ocl::Device::TYPE_GPU))
  {
      cout << "Failed creating the context..." << endl;
      return;
  }

  // In OpenCV 3.0.0 beta, only a single device is detected.
  cout << context.ndevices() << " GPU devices are detected." << endl;
  for (size_t i = 0; i < context.ndevices(); i++)
  {
      cv::ocl::Device device = context.device(i);
      cout << "name                 : " << device.name() << endl;
      cout << "available            : " << device.available() << endl;
      cout << "imageSupport         : " << device.imageSupport() << endl;
      cout << "OpenCL_C_Version     : " << device.OpenCL_C_Version() << endl;
      cout << endl;
  }

}

LineRecognition::~LineRecognition() {
}

void LineRecognition::img_callback(const sensor_msgs::ImageConstPtr& msg) {
  MAT cv_ptr;
  try
  {
    // todo: check if this if we have 8 or 16 greyscale images
    // hardcopies for now, might be possible to process on pointer if fast enough

    // get image and convert directly to UMat
#ifdef USE_UMAT
    cv_ptr = cv_bridge::toCvCopy(msg, "mono16")->image.getUMat(cv::ACCESS_FAST);
#else
    cv_ptr = cv_bridge::toCvCopy(msg, "mono16")->image;
#endif
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if(!cv_ptr.elemSize())
  {
    ROS_WARN("Empty image received, skipping!");
    return;
  }



  /// define some processing constants
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;
  MAT grad;


  /// Blur image
  MAT img_blurred;
  cv::GaussianBlur( cv_ptr, img_blurred, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );


  /// Generate grad_x and grad_y
  MAT grad_x, abs_grad_x;
  MAT grad_y, abs_grad_y;

  /// Gradient X
  cv::Sobel( img_blurred, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
  cv::convertScaleAbs( grad_x, abs_grad_x );

  /// Gradient Y
  cv::Sobel( img_blurred, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
  cv::convertScaleAbs( grad_y, abs_grad_y );

  /// Total Gradient (approximate)
  cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

  /// Create window
  //cv::namedWindow( "Sobel", CV_WINDOW_AUTOSIZE );
  //cv::imshow( "Sobel", grad );
}

} // end drive_ros_image_recognition namespace
