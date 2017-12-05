#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <chrono>
#include <pluginlib/class_list_macros.h>
#include "drive_ros_image_recognition/line_detection.h"
#include "drive_ros_image_recognition/geometry_common.h"

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

  debugImgPub_ = imageTransport_.advertise("line_detection/debug_image", 10);

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
  std::vector<Line> filteredLines;

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
    Line sl(imagePoints.at(i), imagePoints.at(i + 1), worldPoints.at(i), worldPoints.at(i + 1));
    filteredLines.push_back(sl);
#ifdef DRAW_DEBUG
//    cv::line(allLinesImg, sl.iP1_, sl.iP2_, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
#endif
  }

  // Create a dummy middle line, in case we do not have on from the previous frame
  worldPoints.clear();
  imagePoints.clear();
  if(currentLane_.empty()) {
    ROS_INFO_STREAM("dummy guess");
    worldPoints.push_back(cv::Point2f(0.24, 0.5));
    worldPoints.push_back(cv::Point2f(0.4, 0.5));
    worldPoints.push_back(cv::Point2f(0.55, 0.5));
    worldPoints.push_back(cv::Point2f(0.70, 0.5));
    if(image_operator_.worldToImage(worldPoints, imagePoints)) {
      for(size_t i = 0; i < imagePoints.size() - 1; i++)
        currentLane_.push_back(Line(imagePoints.at(i), imagePoints.at(i + 1), worldPoints.at(i), worldPoints.at(i + 1)));
    } else {
      // no guess, means no detection
      return;
    }
  }

#ifdef DRAW_DEBUG
  // Draw our "guess" (previous middle line)
//  for(size_t i = 0; i < currentLane_.size(); i++) {
//    cv::line(outputImg, currentLane_.at(i).iP1_, currentLane_.at(i).iP2_, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
//  }
// todo: we could draw the search corridor here
#endif

  // Use the guess to classify the lines
  std::vector<Line> leftLine, middleLine, rightLine;
  for(Line sl : filteredLines) {
    Line segment = currentLane_.at(0);
      bool isInSegment = false;
      // find the corresponding segment
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
        if(std::abs(sl.getAngle() - segment.getAngle()) < lineAngle_) {
          // lane width is 0.35 to 0.45 [m]
          // todo: distance should be calculated based on orthogonal distance. currently this leads to problems in curves
          auto absDistanceToMidLine = std::abs(sl.wMid_.y - segment.wMid_.y);
          if(absDistanceToMidLine < (lineWidth_ / 2)) {
            middleLine.push_back(sl);
#ifdef DRAW_DEBUG
            cv::line(outputImg, sl.iP1_, sl.iP2_, cv::Scalar(0, 255), 2, cv::LINE_AA);
#endif
          } else if((sl.wMid_.y < segment.wMid_.y) && (std::abs(absDistanceToMidLine - lineWidth_) < lineVar_)) {
            rightLine.push_back(sl);
#ifdef DRAW_DEBUG
            cv::line(outputImg, sl.iP1_, sl.iP2_, cv::Scalar(255), 2, cv::LINE_AA);
#endif
          } else if((sl.wMid_.y > segment.wMid_.y) && (std::abs(absDistanceToMidLine - lineWidth_) < lineVar_)) {
            leftLine.push_back(sl);
#ifdef DRAW_DEBUG
            cv::line(outputImg, sl.iP1_, sl.iP2_, cv::Scalar(255), 2, cv::LINE_AA);
#endif
          }
        } else {
          // todo: here we still have to check the distance from our lane
#ifdef DRAW_DEBUG
          cv::line(outputImg, sl.iP1_, sl.iP2_, cv::Scalar(255, 255), 2, cv::LINE_AA);
#endif
        }
      }
  }

  if(middleLine.empty()) {
    ROS_INFO_STREAM("no middle line found");
    currentLane_.clear();
    debugImgPub_.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, outputImg).toImageMsg());
    return;
  }

  // middle line is 0.2m long and then interrupted for 0.2m
  // sort our middle lines based on wP1_.x of line
  std::sort(middleLine.begin(), middleLine.end(), [](Line &a, Line &b){ return a.wP1_.x < b.wP1_.x; });
  // build bounding boxes around group of lines, where lines in group are closer than 0.3m to each other
  float currentMinX = middleLine.at(0).wP1_.x;
  std::vector<cv::Point2f> cPts, newMidLinePts;
  for(auto l : middleLine) {
    if(l.wP1_.x > (currentMinX + 0.3)) {
      auto rect = cv::minAreaRect(cPts);
      cv::Point2f a, b;
      cv::Point2f vertices2f[4];
      rect.points(vertices2f);

      currentMinX = l.wP1_.x;

      // todo: this should be in a function, since we are using it again after the for loop
      // use the long side as line
      if(std::abs(vertices2f[0].x - vertices2f[1].x) > std::abs(vertices2f[1].x - vertices2f[2].x)) {
        a = vertices2f[0];
        b = vertices2f[1];
      } else {
        a = vertices2f[1];
        b = vertices2f[2];
      }
      // ensures the right order of the points in newMidLinePts vector
      if(a.x > b.x) {
        newMidLinePts.push_back(b);
        newMidLinePts.push_back(a);
      } else {
        newMidLinePts.push_back(a);
        newMidLinePts.push_back(b);
      }

      cPts.clear();
    }
    cPts.push_back(l.wP1_);
    cPts.push_back(l.wP2_);
  }

  // finish the bounding box building step
  if(!cPts.empty()) {
    auto rect = cv::minAreaRect(cPts);
    cv::Point2f vertices2f[4];
    rect.points(vertices2f);
    cv::Point2f a, b;

    // use the long side as line
    if(std::abs(vertices2f[0].x - vertices2f[1].x) > std::abs(vertices2f[1].x - vertices2f[2].x)) {
      a = vertices2f[0];
      b = vertices2f[1];
    } else {
      a = vertices2f[1];
      b = vertices2f[2];
    }
    // ensures the right order of the points in newMidLinePts vector
    if(a.x > b.x) {
      newMidLinePts.push_back(b);
      newMidLinePts.push_back(a);
    } else {
      newMidLinePts.push_back(a);
      newMidLinePts.push_back(b);
    }
  }

  // build the middle line from out points
  // extend the last line to the front
  auto wPt1 = newMidLinePts.at(newMidLinePts.size() - 2);
  auto wPt2 = newMidLinePts.at(newMidLinePts.size() - 1);
  auto dir = wPt2 - wPt1;
  auto dirLen = sqrt(dir.x * dir.x + dir.y * dir.y);

  auto newPt = wPt2 + (dir * (0.6 / dirLen));
  if(newPt.x < maxViewRange_)
    newMidLinePts.push_back(newPt);

  // extend first line to the back
  wPt1 = newMidLinePts.at(0);
  wPt2 = newMidLinePts.at(1);
  dir = wPt2 - wPt1;
  dirLen = sqrt(dir.x * dir.x + dir.y * dir.y);
  auto distToImageBoundary = wPt1.x - 0.24; // 0.24 is known. todo: magic numbers are not nice, put it in config
  if(distToImageBoundary > 0.1) { // distance is positive
    // dir, distToImageBoundary, dirLen are all positive
    newPt = wPt1 - (dir * distToImageBoundary / dirLen);
    newMidLinePts.push_back(newPt);
  }

  // sort the world points based on the distance to the car
  std::sort(newMidLinePts.begin(), newMidLinePts.end(), [](cv::Point2f &a, cv::Point2f &b) { return a.x < b.x; });

  // convert points
  imagePoints.clear();
  image_operator_.worldToImage(newMidLinePts, imagePoints);
  // create new guess
  currentLane_.clear();
  for(size_t i = 1; i < imagePoints.size(); i++) {
    auto a = imagePoints.at(i - 1);
    auto b = imagePoints.at(i);
    // this makes the line coordinates inconsistant, but this should not be a serious problem
    if(a.y > imgHeight)
      a.y = imgHeight - 1;
    if(b.y > imgHeight)
      b.y = imgHeight - 1;

    currentLane_.push_back(Line(a, b, newMidLinePts.at(i - 1), newMidLinePts.at(i)));
  }

  // validate our new guess
  if(!currentLane_.empty()) {
    if(currentLane_.at(currentLane_.size() - 1).wP2_.x < 0.5) {
      // our guess is too short, it is better to use the default one
      currentLane_.clear();
    } else if((currentLane_.at(0).getAngle() < 1.0) || (currentLane_.at(0).getAngle() > 2.6)) {
      // the angle of the first segment is weird (probably a workaround for now, why can this happen?)
      // if we have more than one segment, then we just delete the first
      if(currentLane_.size() > 1) {
        currentLane_.erase(currentLane_.begin());
      } else {
        currentLane_.clear();
      }
    }

    // angles between two connected segments should be plausible
    bool done = false;
    for(size_t i = 1; i < currentLane_.size() && !done; i++) {
      if(std::abs(currentLane_.at(i - 1).getAngle() - currentLane_.at(i).getAngle()) > 0.8) {
        // 0.8 = 45 degree
        // todo: clear whole line or just to this segment?
        currentLane_.erase(currentLane_.begin() + i, currentLane_.end());
        ROS_INFO_STREAM("clipped line because of angle difference");
        done = true;
      } else if(currentLane_.at(i).wP2_.x > maxViewRange_) {
        ROS_INFO_STREAM("clipped line because of maxViewRange_");
        currentLane_.erase(currentLane_.begin() + i, currentLane_.end());
        done = true;
      } else if(currentLane_.at(i).getLength() > 0.7) {
        ROS_INFO_STREAM("clipped line because of length");
        currentLane_.erase(currentLane_.begin() + i, currentLane_.end());
        done = true;
      }
    }
    // draw the new (validated) guess
    for(auto l : currentLane_) {
      cv::line(outputImg, l.iP1_, l.iP2_, cv::Scalar(255,0,255), 2, cv::LINE_AA);
      cv::circle(outputImg, l.iP1_, 5, cv::Scalar(255,0,255), 2);
      cv::circle(outputImg, l.iP2_, 5, cv::Scalar(255,0,255), 2);
    }
  }

  // todo: move old guess with odometry and compare old and new guess

  debugImgPub_.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, outputImg).toImageMsg());

  // todo: publish points to road topic
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
  lineAngle_ = config.lineAngle;
  lineVar_ = config.lineVar;
  maxViewRange_ = config.maxSenseRange;
  cannyThreshold_ = config.cannyThreshold;
  houghThresold_ = config.houghThreshold;
  houghMinLineLen_ = config.houghMinLineLen;
  houghMaxLineGap_ = config.houghMaxLineGap;
}

} // namespace drive_ros_image_recognition
