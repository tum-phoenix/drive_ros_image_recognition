#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <chrono>
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
  , onRightLine(true)
//  #ifdef DRAW_DEBUG
//  , debugImage(0, 0, CV_8UC1)
//  #endif
{
}

StreetCrossingDetection::~StreetCrossingDetection() {
}

bool StreetCrossingDetection::init() {
  // dynamic reconfigure
  dsrv_server_.setCallback(dsrv_cb_);

  // image operations
//  if(!transform_helper_.init())
//    ROS_INFO_STREAM("Init transform_helper failed");
//  image_operator_ = ImageOperator(transform_helper_);

  //subscribe to camera image
  imageSubscriber_ = imageTransport_.subscribe("img_in", 10, &StreetCrossingDetection::imageCallback, this);
  ROS_INFO_STREAM("Subscribed image transport to topic " << imageSubscriber_.getTopic());

  // subscribe to warped image
//  warpedImageSubscriber_ = imageTransport_.subscribe("warped_in", 10, &StreetCrossingDetection::warpedImageCallback, this);
//  ROS_INFO_STREAM("Subscribed image transport to topic " << warpedImageSubscriber_.getTopic());

  // publish road lane
//  line_output_pub_ = pnh_.advertise<drive_ros_msgs::RoadLane>("line_out",10);

#ifdef PUBLISH_DEBUG
  debugImagePublisher_ = imageTransport_.advertise("/street_crossing/debug_image", 10);
  ROS_INFO_STREAM("Publishing debug image on " << debugImagePublisher_.getTopic());
#endif

  // common image operations
  image_operator_.init();
  std::string world_frame("/rear_axis_middle_ground");
  if (!pnh_.getParam("world_frame", world_frame))
    ROS_WARN_STREAM("Parameter 'world_frame' not found, using default: "<<world_frame);
  std::string camera_frame("/camera_optical");
  if (!pnh_.getParam("camera_frame", camera_frame))
    ROS_WARN_STREAM("Parameter 'camera_frame' not found, using default: "<<camera_frame);
  image_operator_.setCameraFrame(camera_frame);
  image_operator_.setWorldFrame(world_frame);

  return true;
}

void StreetCrossingDetection::warpedImageCallback(const sensor_msgs::ImageConstPtr& warpedImageIn) {
//  auto imgPtr = convertImageMessage(warpedImageIn);
//  cv::cvtColor(*imgPtr, warpedImg_, CV_GRAY2RGB);

//    cv::namedWindow("warped image", CV_WINDOW_NORMAL);
//    cv::imshow("warped image", warpedImg_);
//  ROS_INFO_STREAM("received a warped image");
}

void StreetCrossingDetection::imageCallback(const sensor_msgs::ImageConstPtr &imgIn) {
  currentImage_ = convertImageMessage(imgIn);

//  findStartline();
//  auto t_start = std::chrono::high_resolution_clock::now();
  findLane();
//  auto t_end = std::chrono::high_resolution_clock::now();
//  ROS_INFO_STREAM("Cycle time: " << (std::chrono::duration<double, std::milli>(t_end-t_start).count()) << "ms");
}

void StreetCrossingDetection::findLane() {
  cv::Mat debugImg, outputImg;
  cv::cvtColor(*currentImage_, outputImg, CV_GRAY2RGB);
  cv::Mat laneImg = cv::Mat::zeros(outputImg.rows, outputImg.cols, outputImg.type());

  // Blur the debugImage and find edges with Canny
  cv::GaussianBlur(*currentImage_, debugImg, cv::Size(15, 15), 0, 0);
  cv::Canny(debugImg, debugImg, cannyThreshold_, cannyThreshold_ * 3, 3);

  // Zero out the vehicle
  int w = debugImg.cols;
  int h = debugImg.rows;
  cv::Point vehiclePolygon[1][4];
  vehiclePolygon[0][0] = cv::Point(w * .35, h);
  vehiclePolygon[0][1] = cv::Point(w * .65, h);
  vehiclePolygon[0][2] = cv::Point(w * .65, h * .8);
  vehiclePolygon[0][3] = cv::Point(w * .35, h * .8);
  const cv::Point* ppt[1] = { vehiclePolygon[0] };
  int npt[] = { 4 };
  cv::fillPoly(debugImg, ppt, npt, 1, cv::Scalar(), cv::LINE_8);

  // If we dont have a lane from last step (or this is the first search), use straight line forward
  if(currentLaneMid.empty()) {
    currentLaneMid.push_back(StreetLine(cv::Point(w/2, h), cv::Point(w/2, 0)));
  }

  // Display the polygon which will be zeroed out
//  cv::line(outputImg, vehiclePolygon[0][0], vehiclePolygon[0][1], cv::Scalar(255), 2, cv::LINE_AA);
//  cv::line(outputImg, vehiclePolygon[0][1], vehiclePolygon[0][2], cv::Scalar(255), 2, cv::LINE_AA);
//  cv::line(outputImg, vehiclePolygon[0][2], vehiclePolygon[0][3], cv::Scalar(255), 2, cv::LINE_AA);
//  cv::line(outputImg, vehiclePolygon[0][3], vehiclePolygon[0][0], cv::Scalar(255), 2, cv::LINE_AA);

  // Get houghlines and draw them on the output image
  std::vector<cv::Vec4i> hLinePoints;
  std::vector<StreetLine> leftLines, midLines, rightLines;
  cv::HoughLinesP(debugImg, hLinePoints, 1, CV_PI / 180, houghThresold_, houghMinLineLen_, houghMaxLineGap_);

  // Assign the hough-lines returned by HoughLinesP to left, mid, or right line
  for(size_t i = 0; i < hLinePoints.size(); i++) {
    auto l = StreetLine(hLinePoints[i]);

    // todo: find current line segment
    StreetLine currentSegment = currentLaneMid.at(0);

    if(currentSegment.mid_.x < l.mid_.x) {
      rightLines.push_back(l);
    } else if((currentSegment.mid_.x - (w * .15)) > l.mid_.x) {
      leftLines.push_back(l);
    } else {
      midLines.push_back(l);
    }


    // (0,0)                (1280,344)
    // (-0.0506457,3.06453) (6.85171,-0.989774)
    cv::Point2f wPt1, wPt2;
//    image_operator_.imageToWorld(l.p1_, wPt1);
//    image_operator_.imageToWorld(l.p2_, wPt2);
    image_operator_.imageToWorld(cv::Point(0, 0), wPt1);
    image_operator_.imageToWorld(cv::Point(w, h), wPt2);
    cv::line(laneImg, wPt1, wPt2, cv::Scalar(255), 2, cv::LINE_AA);
    ROS_INFO_STREAM("Transformed (0,0) to (" << wPt1.x << "," << wPt1.y <<")");
    ROS_INFO_STREAM("Transformed (" << w << "," << h << " to (" << wPt2.x << "," << wPt2.y << ")");
  }


  cv::line(outputImg, cv::Point(w * .15, h), cv::Point(w * .45, 0), cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

  for(auto l : leftLines)
    cv::line(outputImg, l.p1_, l.p2_, cv::Scalar(255), 2, cv::LINE_AA);
  for(auto l : midLines)
    cv::line(outputImg, l.p1_, l.p2_, cv::Scalar(0, 255), 2, cv::LINE_AA);
  for(auto l : rightLines)
    cv::line(outputImg, l.p1_, l.p2_, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

//    auto a = l.getAngle();
//    if((a > (CV_PI/6)) && (a < 2.6)) {
//      // lane line
//      for(size_t j = 0; j < laneLines.size(); j++) {
//        auto d = abs(l.mid_.x - laneLines.at(j).mid_.x);
//        if(d < sameLineThreshold_) {
//          laneLines.at(j).vote++;
//          l.vote++;
//        }
//      }
//      laneLines.push_back(l);
//    } else {
//      // stop line
//      for(size_t j = 0; j < stopLines.size(); j++) {
//        auto d = abs(l.mid_.y - stopLines.at(j).mid_.y);
//        if(d < sameLineThreshold_) {
//          stopLines.at(j).vote++;
//          l.vote++;
//        }
//      }
//      stopLines.push_back(l);
//    }

//    for(auto l : laneLines) {
//      if(l.vote > 0)
//        cv::line(outputImg, l.p1_, l.p2_, cv::Scalar(0,255), 2, cv::LINE_AA);
//      else
//        cv::line(outputImg, l.p1_, l.p2_, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
//    }

//    for(auto l : stopLines) {
//      if(l.vote > 0)
//        cv::line(outputImg, l.p1_, l.p2_, cv::Scalar(255), 2, cv::LINE_AA);
//      else
//        cv::line(outputImg, l.p1_, l.p2_, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
//    }

//      bool added = false;
//      for(size_t j = 0; j < myLines.size(); j++) {
//        // todo: use world distance here with linewidth
//        auto d = cv::norm(l.mid_ - myLines.at(j).mid_);
//        if(d < sameLineThreshold_) {
//          myLines.at(j).vote++;
//          added = true;
//        }
//      }
//      if(!added) {
//        myLines.push_back(l);
//      }
//  }

//  int linesDrawn = 0;
//  for(size_t i = 0; i < myLines.size(); i++) {
//    auto l = myLines.at(i);
//    auto a = l.getAngle();
//    if(l.vote > 0) {
//      linesDrawn++;
//      if((a > (CV_PI/6)) && (a < 2.6)) {
//        // lane marking
//        cv::line(outputImg, l.p1_, l.p2_, cv::Scalar(0,255), 2, cv::LINE_AA);
//      } else {
//        // stop or start line
//        cv::line(outputImg, l.p1_, l.p2_, cv::Scalar(255), 2, cv::LINE_AA);
//      }
//    } else {
//      // discarded line due to votes
//      cv::line(outputImg, l.p1_, l.p2_, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
//    }
//  }

//  ROS_INFO_STREAM("out of " << hLinePoints.size() << " hough lines, " << linesDrawn << " survived");

#ifdef PUBLISH_DEBUG
  debugImagePublisher_.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, outputImg).toImageMsg());
#endif

  cv::namedWindow("debug image", CV_WINDOW_NORMAL);
  cv::imshow("debug image", outputImg);
  cv::namedWindow("lane image", CV_WINDOW_NORMAL);
  cv::imshow("lane image", laneImg);
  cv::waitKey(1);
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
//  debugImagePublisher.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_8UC1, debugImage).toImageMsg());
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

  cannyThreshold_ = config.cannyThreshold;
  houghThresold_ = config.houghThreshold;
  houghMinLineLen_ = config.houghMinLineLen;
  houghMaxLineGap_ = config.houghMaxLineGap;
  sameLineThreshold_ = config.sameLineThreshold;
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
