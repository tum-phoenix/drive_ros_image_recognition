#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include "drive_ros_image_recognition/street_crossing.h"
#include <pluginlib/class_list_macros.h>

namespace drive_ros_image_recognition {

StreetCrossingDetection::StreetCrossingDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh)
  : nh_(nh)
  , pnh_(pnh)
  , imageTransport_(pnh)
  , sobelThreshold_(50)
  , minLineWidthMul_(0.5)
  , maxLineWidthMul_(2.0)
  , lineWidth_(0.4)
  , line_output_pub_()
  , transform_helper_()
  , image_operator_()
  , dsrv_server_()
  , dsrv_cb_(boost::bind(&StreetCrossingDetection::reconfigureCB, this, _1, _2))
  , startLineCounter_(0)
{
}

StreetCrossingDetection::~StreetCrossingDetection() {
}

bool StreetCrossingDetection::init() {
  dsrv_server_.setCallback(dsrv_cb_);

  transform_helper_.init(pnh_);
  image_operator_ = ImageOperator(transform_helper_);

  imageSubscriber_ = imageTransport_.subscribe("img_in", 10, &StreetCrossingDetection::imageCallback, this);
  ROS_INFO_STREAM("Subscriber image transport with topic "<<imageSubscriber_.getTopic());

  line_output_pub_ = pnh_.advertise<drive_ros_msgs::RoadLane>("line_out",10);

#ifdef PUBLISH_DEBUG
  debugImagePublisher = imageTransport_.advertise("/street_crossing/debug_image", 10);
  ROS_INFO("Will publish debug image to /street_crossing/debug_image");
#endif
#ifdef DRAW_DEBUG
  ROS_INFO("Draw debug image");
#endif
  return true;
}

void StreetCrossingDetection::imageCallback(const sensor_msgs::ImageConstPtr &imgIn) {
  currentImage_ = convertImageMessage(imgIn);
  findStartline();
}

bool StreetCrossingDetection::findStartline() {
  SearchLine slRightLane;
//  SearchLine slLeftLane;

  // use search-line to find stop-line
  slRightLane.iStart = cv::Point2f(currentImage_->cols / 2, currentImage_->rows * .4);
  slRightLane.iEnd = cv::Point2f(currentImage_->cols / 2, currentImage_->rows * .8);

//  slLeftLane.iStart = cv::Point2f(currentImage_->cols / 8 * 3, currentImage_->rows * .4);
//  slLeftLane.iEnd = cv::Point2f(currentImage_->cols / 4, currentImage_->rows * .8);
//    slRightLane.iStart = cv::Point2f(currentImage_->cols * .55, currentImage_->rows * .4);
//    slRightLane.iEnd = cv::Point2f(currentImage_->cols * .55, currentImage_->rows * .8);

//    slLeftLane.iStart = cv::Point2f(currentImage_->cols * .45, currentImage_->rows * .4);
//    slLeftLane.iEnd = cv::Point2f(currentImage_->cols * .45, currentImage_->rows * .8);

  // find line using unified header
  search_direction search_dir = search_direction::y;
  search_method search_meth = search_method::sobel;
  std::vector<cv::Point> image_points;
  std::vector<int> line_widths;
  image_operator_.findByLineSearch(slRightLane,*currentImage_,search_dir,search_meth,image_points,line_widths);
//  image_operator_.findByLineSearch(slLeftLane,*currentImage_,search_dir,search_meth,image_points,line_widths);
  if(image_points.empty()) {
    startLineCounter_ = 0;
  } else {
    startLineCounter_++;
    if(image_points.size() > 1) {
      ROS_INFO("Found more than one start/stop line");
    }
  }
#if defined(DRAW_DEBUG) || defined(PUBLISH_DEBUG)
  cv::Mat debug_image;
  cv::cvtColor(*currentImage_, debug_image, CV_GRAY2RGB);
  cv::line(debug_image, slRightLane.iStart, slRightLane.iEnd, cv::Scalar(255, 255, 255), 2);
//  cv::line(debug_image, slLeftLane.iStart, slLeftLane.iEnd, cv::Scalar(255, 255, 255), 2);
  if(startLineCounter_ > 0) {
    for (auto point: image_points) {
      cv::circle(debug_image,point,2,cv::Scalar(0,255,0),3);
    }
  }
#endif
#ifdef DRAW_DEBUG
  cv::namedWindow("Crossing Search debug",CV_WINDOW_NORMAL);
  cv::imshow("Crossing Search debug",debug_image);
  cv::waitKey(1);
#endif
#ifdef PUBLISH_DEBUG
  debugImagePublisher.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_8UC1, debug_image).toImageMsg());
#endif

  // test array of lines
  //  image_operator_.debugPointsImage(*currentImage, search_dir, search_meth);
}

void StreetCrossingDetection::reconfigureCB(drive_ros_image_recognition::LineDetectionConfig& config, uint32_t level){
  image_operator_.setConfig(config);
  sobelThreshold_ = config.sobelThreshold;
  minLineWidthMul_ = config.minLineWidthMul;
  maxLineWidthMul_ = config.maxLineWidthMul;
  lineWidth_ = config.lineWidth;
}

void StreetCrossingDetectionNodelet::onInit() {
  street_crossing_detection_.reset(new StreetCrossingDetection(getNodeHandle(),getPrivateNodeHandle()));
  if (!street_crossing_detection_->init()) {
    ROS_ERROR("StreetCrossing nodelet failed to initialize");
    // nodelet failing will kill the entire loader anyway
    ros::shutdown();
  }
  else {
    ROS_INFO("StreetCrossing detection nodelet succesfully initialized");
  }
}

} // namespace drive_ros_image_recognition

PLUGINLIB_EXPORT_CLASS(drive_ros_image_recognition::StreetCrossingDetectionNodelet, nodelet::Nodelet)
