#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <chrono>
#include "drive_ros_image_recognition/line_detection.h"
#include <pluginlib/class_list_macros.h>

namespace drive_ros_image_recognition {

LineDetection::LineDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh)
  : nh_(nh)
  , pnh_(pnh)
  , imageTransport_(pnh)
  , lineWidth_(0.4)
  , line_output_pub_()
  , image_operator_()
  , dsrv_server_()
  , dsrv_cb_(boost::bind(&LineDetection::reconfigureCB, this, _1, _2))
{
}

LineDetection::~LineDetection() {
}

///
/// \brief LineDetection::init
/// Sets dynamic reconfigure server, subscribes to camera image, inits image_operator.
/// \return true for success, false for failure.
///
bool LineDetection::init() {
  // dynamic reconfigure
  dsrv_server_.setCallback(dsrv_cb_);

  //subscribe to camera image
  imageSubscriber_ = imageTransport_.subscribe("img_in", 10, &LineDetection::imageCallback, this);
  ROS_INFO_STREAM("Subscribed image transport to topic " << imageSubscriber_.getTopic());

  // common image operations
  if(!image_operator_.init()) {
    ROS_WARN_STREAM("Failed to init image_operator");
    return false;
  }

  return true;
}

///
/// \brief LineDetection::imageCallback
/// Called for incoming camera image. Extracts the image from the incoming message.
/// \param imgIn the incoming camera image message.
///
void LineDetection::imageCallback(const sensor_msgs::ImageConstPtr &imgIn) {
  currentImage_ = convertImageMessage(imgIn);

//  auto t_start = std::chrono::high_resolution_clock::now();
  findLane();
//  auto t_end = std::chrono::high_resolution_clock::now();
//  ROS_INFO_STREAM("Cycle time: " << (std::chrono::duration<double, std::milli>(t_end-t_start).count()) << "ms");
}

///
/// \brief LineDetection::findLane
/// Uses the current image to find the street marking lanes.
///
void LineDetection::findLane() {
  cv::Mat processingImg;
#ifdef DRAW_DEBUG
  cv::Mat outputImg;
  cv::cvtColor(*currentImage_, outputImg, CV_GRAY2RGB);
  cv::Mat allLinesImg = outputImg.clone();
#endif

  // Blur the image and find edges with Canny
  cv::GaussianBlur(*currentImage_, processingImg, cv::Size(15, 15), 0, 0);
  cv::Canny(processingImg, processingImg, cannyThreshold_, cannyThreshold_ * 3, 3);

  // Zero out the vehicle
  int imgWidth = processingImg.cols;
  int imgHeight = processingImg.rows;
  cv::Point vehiclePolygon[1][4];
  vehiclePolygon[0][0] = cv::Point(imgWidth * .35, imgHeight);
  vehiclePolygon[0][1] = cv::Point(imgWidth * .65, imgHeight);
  vehiclePolygon[0][2] = cv::Point(imgWidth * .65, imgHeight * .8);
  vehiclePolygon[0][3] = cv::Point(imgWidth * .35, imgHeight * .8);
  const cv::Point* polygonStarts[1] = { vehiclePolygon[0] };
  int polygonLengths[] = { 4 };
  cv::fillPoly(processingImg, polygonStarts, polygonLengths, 1, cv::Scalar(), cv::LINE_8);

#ifdef DRAW_DEBUG
  // Display the polygon which will be zeroed out
//  cv::line(outputImg, vehiclePolygon[0][0], vehiclePolygon[0][1], cv::Scalar(255), 2, cv::LINE_AA);
//  cv::line(outputImg, vehiclePolygon[0][1], vehiclePolygon[0][2], cv::Scalar(255), 2, cv::LINE_AA);
//  cv::line(outputImg, vehiclePolygon[0][2], vehiclePolygon[0][3], cv::Scalar(255), 2, cv::LINE_AA);
//  cv::line(outputImg, vehiclePolygon[0][3], vehiclePolygon[0][0], cv::Scalar(255), 2, cv::LINE_AA);
#endif

  // Get houghlines
  std::vector<cv::Vec4i> hLinePoints;
  std::vector<StreetLine> filteredLines;

  cv::HoughLinesP(processingImg, hLinePoints, 1, CV_PI / 180, houghThresold_, houghMinLineLen_, houghMaxLineGap_);

  // Extract points for houghLines and convert to worldCoordinates
  std::vector<cv::Point2f> imagePoints, worldPoints;
  for(size_t i = 0; i < hLinePoints.size(); i++) {
    cv::Vec4i currentPoints = hLinePoints.at(i);
    imagePoints.push_back(cv::Point(currentPoints[0], currentPoints[1]));
    imagePoints.push_back(cv::Point(currentPoints[2], currentPoints[3]));
  }

  if(!image_operator_.imageToWorld(imagePoints, worldPoints)) {
    // failed converting points 
    return;
  }

  // Filter the lines
  for(size_t i = 0; i < worldPoints.size(); i += 2) {
    StreetLine sl(imagePoints.at(i), imagePoints.at(i + 1), worldPoints.at(i), worldPoints.at(i + 1));
    filteredLines.push_back(sl);
#ifdef DRAW_DEBUG
    cv::line(allLinesImg, sl.iP1_, sl.iP2_, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
#endif
  }

  // Create a dummy middle line, in case we do not have on from the previous frame
  worldPoints.clear();
  imagePoints.clear();
  if(currentLane_.empty()) {
    worldPoints.push_back(cv::Point2f(0.24, 0.5));
    worldPoints.push_back(cv::Point2f(0.4, 0.5));
    worldPoints.push_back(cv::Point2f(0.55, 0.5));
    worldPoints.push_back(cv::Point2f(0.70, 0.5));
    if(image_operator_.worldToImage(worldPoints, imagePoints)) {
      for(size_t i = 0; i < imagePoints.size() - 1; i++)
        currentLane_.push_back(StreetLine(imagePoints.at(i), imagePoints.at(i + 1), worldPoints.at(i), worldPoints.at(i + 1)));
    } else {
      // no guess, means no detection
      return;
    }
  }

#ifdef DRAW_DEBUG
  // Draw our "guess" (previous middle line)
  for(size_t i = 0; i < currentLane_.size(); i++) {
    cv::line(outputImg, currentLane_.at(i).iP1_, currentLane_.at(i).iP2_, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
  }
#endif

  // Use the guess to classify the lines
  std::vector<StreetLine> leftLine, middleLine, rightLine;
  for(StreetLine sl : filteredLines) {
    StreetLine segment = currentLane_.at(0);
      bool isInSegment = false;
      // find the coresponding segment
      // todo: this only uses the x-coordinate right now, improve this
      for(size_t i = 0; (i < currentLane_.size()) && !isInSegment; i++) {
        segment = currentLane_.at(i);
        if(sl.wMid_.x > segment.wP1_.x) {
          if(sl.wMid_.x < segment.wP2_.x) {
            isInSegment = true;
          }
        } else if(sl.wMid_.x > segment.wP2_.x) {
          isInSegment = true;
        }
      }

      if(isInSegment) {
        // classify the line
        // lane width is 0.35 to 0.45 [m]
        auto distanceToMidLine = sl.wMid_.y - segment.wMid_.y;
        if(std::abs(distanceToMidLine) < 0.175) {
          middleLine.push_back(sl);
#ifdef DRAW_DEBUG
          cv::line(outputImg, sl.iP1_, sl.iP2_, cv::Scalar(0, 255), 2, cv::LINE_AA);
#endif
        } else if((sl.wMid_.y < segment.wMid_.y) && (std::abs(distanceToMidLine) < .6)) {
          rightLine.push_back(sl);
#ifdef DRAW_DEBUG
          cv::line(outputImg, sl.iP1_, sl.iP2_, cv::Scalar(255), 2, cv::LINE_AA);
#endif
        } else if(distanceToMidLine < .6) {
          leftLine.push_back(sl);
#ifdef DRAW_DEBUG
          cv::line(outputImg, sl.iP1_, sl.iP2_, cv::Scalar(255), 2, cv::LINE_AA);
#endif
        } else {
#ifdef DRAW_DEBUG
          cv::line(outputImg, sl.iP1_, sl.iP2_, cv::Scalar(255, 255), 2, cv::LINE_AA);
#endif
        }
      }
  }

  // Use the new middle line as our new guess
//  currentLane_.clear();
////  for(auto l : middleLine)
////    currentLane_.push_back(l);

//  // sort the middle lines based on the front distance of the vehicle
////  std::sort(middleLine.begin(), middleLine.end(), [](const StreetLine& a, const StreetLine& b){ return a.wMid_.x < b.wMid_.x; });
//  std::vector<cv::Point2f> guessImgPoints, guessWorldPoints;
//  for(auto lineIt = middleLine.begin(); lineIt != middleLine.end(); ++lineIt) {
//    guessWorldPoints.push_back(lineIt->wP1_);
//    guessWorldPoints.push_back(lineIt->wP2_);
//  }
//  std::sort(guessWorldPoints.begin(), guessWorldPoints.end(), [](const cv::Point2f& a, const cv::Point2f& b){ return a.x < b.x; });
//  // now add another lane as the last one to extend our current lane
//  cv::Point lastLineDirection = guessWorldPoints.at(guessWorldPoints.size() - 1) - guessWorldPoints.at(guessWorldPoints.size() - 2);
//  auto normFactor = 1 / cv::norm(lastLineDirection);
//  ROS_INFO_STREAM("normFactor=" << normFactor);

//  if((guessWorldPoints.size() > 0) && (image_operator_.worldToImage(guessWorldPoints, guessImgPoints))) {
//    for(size_t i = 1; i < guessImgPoints.size(); i++) {
//      currentLane_.push_back(StreetLine(guessImgPoints.at(i - 1), guessImgPoints.at(i), guessWorldPoints.at(i - 1), guessWorldPoints.at(i)));
//    }

//  }
  

        // angle classification
//      if((a > (CV_PI/6)) && (a < 2.6)) {
//        // lane marking
//        cv::line(outputImg, l.iP1_, l.iP2_, cv::Scalar(0,255), 2, cv::LINE_AA);
//      } else {
//        // stop or start line
//        cv::line(outputImg, l.iP1_, l.iP2_, cv::Scalar(255), 2, cv::LINE_AA);
//      }

#ifdef DRAW_DEBUG
  cv::namedWindow("All Lines", CV_WINDOW_NORMAL);
  cv::imshow("All Lines", allLinesImg);
  cv::namedWindow("Debug Image", CV_WINDOW_NORMAL);
  cv::imshow("Debug Image", outputImg);
  cv::waitKey(1);
#endif
}

///
/// \brief LineDetection::reconfigureCB
/// Used by the dynamic reconfigure server
/// \param config
/// \param level
///
void LineDetection::reconfigureCB(drive_ros_image_recognition::LineDetectionConfig& config, uint32_t level){
  image_operator_.setConfig(config);
  lineWidth_ = config.lineWidth;
  cannyThreshold_ = config.cannyThreshold;
  houghThresold_ = config.houghThreshold;
  houghMinLineLen_ = config.houghMinLineLen;
  houghMaxLineGap_ = config.houghMaxLineGap;
}

} // namespace drive_ros_image_recognition
