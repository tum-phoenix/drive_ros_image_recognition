#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <chrono>
#include <pluginlib/class_list_macros.h>
#include "drive_ros_image_recognition/line_detection.h"
#include "drive_ros_image_recognition/geometry_common.h"
#include "drive_ros_msgs/RoadLine.h"

namespace drive_ros_image_recognition {

LineDetection::LineDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh)
  : nh_(nh)
  , pnh_(pnh)
  , imageTransport_(pnh)
  , lineWidth_(0.4)
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
  // todo: topics as params
  // dynamic reconfigure
  dsrv_server_.setCallback(dsrv_cb_);

  //subscribe to camera image
  imageSubscriber_ = imageTransport_.subscribe("img_in", 10, &LineDetection::imageCallback, this);
  ROS_INFO_STREAM("Subscribed image transport to topic " << imageSubscriber_.getTopic());

  line_output_pub_ = nh_.advertise<drive_ros_msgs::RoadLine>("roadLine", 10);
  ROS_INFO_STREAM("Advertising road line on " << line_output_pub_.getTopic());

#ifdef PUBLISH_DEBUG
  debugImgPub_ = imageTransport_.advertise("debug_image", 10);
  ROS_INFO_STREAM("publishing debug image on topic " << debugImgPub_.getTopic());
#endif

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
#ifdef PUBLISH_DEBUG
  cv::Mat outputImg;
  cv::cvtColor(*currentImage_, outputImg, CV_GRAY2RGB);
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

#ifdef PUBLISH_DEBUG
  // Display the polygon which will be zeroed out
//  cv::line(outputImg, vehiclePolygon[0][0], vehiclePolygon[0][1], cv::Scalar(255), 2, cv::LINE_AA);
//  cv::line(outputImg, vehiclePolygon[0][1], vehiclePolygon[0][2], cv::Scalar(255), 2, cv::LINE_AA);
//  cv::line(outputImg, vehiclePolygon[0][2], vehiclePolygon[0][3], cv::Scalar(255), 2, cv::LINE_AA);
//  cv::line(outputImg, vehiclePolygon[0][3], vehiclePolygon[0][0], cv::Scalar(255), 2, cv::LINE_AA);
#endif

  // Get houghlines
  std::vector<cv::Vec4i> hLinePoints;
  std::vector<Line> houghLines;

  cv::HoughLinesP(processingImg, hLinePoints, 1, CV_PI / 180, houghThresold_, houghMinLineLen_, houghMaxLineGap_);

  // Extract points for houghLines and convert to world-coordinates
  std::vector<cv::Point2f> imagePoints, worldPoints;
  for(size_t i = 0; i < hLinePoints.size(); i++) {
    cv::Vec4i currentPoints = hLinePoints.at(i);
    imagePoints.push_back(cv::Point(currentPoints[0], currentPoints[1]));
    imagePoints.push_back(cv::Point(currentPoints[2], currentPoints[3]));
  }
  image_operator_.imageToWorld(imagePoints, worldPoints);

  // Build lines from points
  for(size_t i = 0; i < worldPoints.size(); i += 2) {
    houghLines.push_back(Line(imagePoints.at(i), imagePoints.at(i + 1), worldPoints.at(i), worldPoints.at(i + 1)));
  }

  // Create a static guess, in case we do not have on from the previous frame
  // todo: we should only do, when the car starts in the start box. Otherwise we can get in trouble
  worldPoints.clear();
  imagePoints.clear();
  if(currentGuess_.empty()) {
    ROS_INFO_STREAM("dummy guess");
    worldPoints.push_back(cv::Point2f(0.24, 0.5));
    worldPoints.push_back(cv::Point2f(0.4, 0.5));
    worldPoints.push_back(cv::Point2f(0.55, 0.5));
    worldPoints.push_back(cv::Point2f(0.70, 0.5));
    image_operator_.worldToImage(worldPoints, imagePoints);
    for(size_t i = 1; i < imagePoints.size(); i++)
      currentGuess_.push_back(Line(imagePoints.at(i - 1), imagePoints.at(i), worldPoints.at(i - 1), worldPoints.at(i)));
  }

#ifdef PUBLISH_DEBUG
  // Draw our guess
  for(size_t i = 0; i < currentGuess_.size(); i++) {
    cv::line(outputImg, currentGuess_.at(i).iP1_, currentGuess_.at(i).iP2_, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
  }
#endif

  // Use the guess to classify the lines
  std::vector<Line> leftLine, middleLine, rightLine;
  for(Line sl : houghLines) {
    Line segment = currentGuess_.at(0);
      bool isInSegment = false;
      // find the corresponding segment
      // todo: this only uses the x-coordinate right now, improve this
      for(size_t i = 0; (i < currentGuess_.size()) && !isInSegment; i++) {
        segment = currentGuess_.at(i);
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
          // todo: distance should be calculated based on orthogonal distance.
          // the current approach leads to problems in curves
          auto absDistanceToMidLine = std::abs(sl.wMid_.y - segment.wMid_.y);
          if(absDistanceToMidLine < (lineWidth_ / 2)) {
            middleLine.push_back(sl);
#ifdef PUBLISH_DEBUG
            cv::line(outputImg, sl.iP1_, sl.iP2_, cv::Scalar(0, 255), 2, cv::LINE_AA);
#endif
          } else if((sl.wMid_.y < segment.wMid_.y) && (std::abs(absDistanceToMidLine - lineWidth_) < lineVar_)) {
            rightLine.push_back(sl);
#ifdef PUBLISH_DEBUG
            cv::line(outputImg, sl.iP1_, sl.iP2_, cv::Scalar(255), 2, cv::LINE_AA);
#endif
          } else if((sl.wMid_.y > segment.wMid_.y) && (std::abs(absDistanceToMidLine - lineWidth_) < lineVar_)) {
            leftLine.push_back(sl);
#ifdef PUBLISH_DEBUG
            cv::line(outputImg, sl.iP1_, sl.iP2_, cv::Scalar(255), 2, cv::LINE_AA);
#endif
          }
        } else {
          // todo: classify the line (stop, start, ...)
#ifdef PUBLISH_DEBUG
          cv::line(outputImg, sl.iP1_, sl.iP2_, cv::Scalar(255, 255), 2, cv::LINE_AA);
#endif
        }
      }
  }

  if(middleLine.empty()) {
    // todo: handle this better
    ROS_INFO_STREAM("no middle line found");
    currentGuess_.clear();
#ifdef PUBLISH_DEBUG
    debugImgPub_.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, outputImg).toImageMsg());
#endif
    return;
  }

  // transform old line based on odometry
//  tf::StampedTransform transform;
//  try{
//    tfListener_.lookupTransform("/odometry", "/rear_axis_middle_ground", ros::Time(0), transform);
//  }
//  catch (tf::TransformException ex){
//    ROS_ERROR("%s",ex.what());
//    ros::Duration(1.0).sleep();
//  }
//  cv::Point2f newPoint(transform.getOrigin().x(), transform.getOrigin().y());
//  //  auto xTransform = newPoint.x - oldPoint_.x;
//  //  auto yTransform = newPoint.y - oldPoint_y;
//  auto ptsDifference = newPoint - oldPoint_;
//  ROS_INFO_STREAM(ptsDifference.x << "," << ptsDifference.y);
//  if(ptsDifference.x != 0.0){
//    oldPoint_ = newPoint;
//    std::vector<cv::Point2f> oldTransformedLine;
//    for(auto seg : currentLane_) {
//      oldTransformedLine.push_back(cv::Point2f(seg.wP1_ + ptsDifference));
//      oldTransformedLine.push_back(cv::Point2f(seg.wP2_ + ptsDifference));
//    }


//    imagePoints.clear();
//    image_operator_.worldToImage(oldTransformedLine, imagePoints);
//    for(auto i = 1; i < (imagePoints.size() - 1); i++)
//      cv::line(outputImg, imagePoints.at(i - 1), imagePoints.at(i), cv::Scalar(255,255), 2);
//    for(auto i = 0; i < currentLane_.size(); i++) {
//      auto p1 = imagePoints.at(i * 2);
//      auto p2 = imagePoints.at((i * 2) + 1);
//      cv::circle(outputImg, currentLane_.at(i).iP1_, 5, cv::Scalar(124,124), 3);
//      cv::circle(outputImg, p1, 5, cv::Scalar(255,255), 3);
//      cv::line(outputImg, currentLane_.at(i).iP1_, p1, cv::Scalar(124,124), 2);
//      cv::circle(outputImg, currentLane_.at(i).iP2_, 5, cv::Scalar(124,124), 3);
//      cv::circle(outputImg, p2, 5, cv::Scalar(255,255), 3);
//      cv::line(outputImg, currentLane_.at(i).iP2_, p2, cv::Scalar(124,124), 2);
//    }
//  }

  // middle line is 0.2m long and then interrupted for 0.2m
  // todo: this should be checked
  // sort our middle lines based on wP1_.x of line
  std::sort(middleLine.begin(), middleLine.end(), [](Line &a, Line &b){ return a.wP1_.x < b.wP1_.x; });
  // build bounding boxes around group of lines, where lines in group are closer than 0.25m to each other
  float currentMinX = middleLine.at(0).wP1_.x;
  std::vector<cv::Point2f> cPts, newMidLinePts;
  for(auto l : middleLine) {
    if(l.wP1_.x > (currentMinX + 0.25)) {
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
  // the image starts 0.24m from the rear_axis_middle_ground. todo: sould be in config
  auto distToImageBoundary = wPt1.x - 0.24;
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
  // create new guess and middle line
  currentGuess_.clear();
  for(size_t i = 1; i < imagePoints.size(); i++) {
    auto a = imagePoints.at(i - 1);
    auto b = imagePoints.at(i);
    // this makes the line coordinates inconsistant, but this should not be a serious problem
    if(a.y > imgHeight)
      a.y = imgHeight - 1;
    if(b.y > imgHeight)
      b.y = imgHeight - 1;

    currentGuess_.push_back(Line(a, b, newMidLinePts.at(i - 1), newMidLinePts.at(i)));
  }

  // validate our new guess
  if(!currentGuess_.empty()) {
    if(currentGuess_.at(currentGuess_.size() - 1).wP2_.x < 0.5) {
      // our guess is too short, it is better to use the default one
      currentGuess_.clear();
    } else if((currentGuess_.at(0).getAngle() < 1.0) || (currentGuess_.at(0).getAngle() > 2.6)) {
      // the angle of the first segment is weird (probably a workaround for now, why can this happen?)
      // if we have more than one segment, then we just delete the first
      if(currentGuess_.size() > 1) {
        currentGuess_.erase(currentGuess_.begin());
      } else {
        // otherwise throw the guess away
        currentGuess_.clear();
      }
    }


    bool done = false;
    for(size_t i = 1; i < currentGuess_.size() && !done; i++) {
      // angles between two connected segments should be plausible
      if(std::abs(currentGuess_.at(i - 1).getAngle() - currentGuess_.at(i).getAngle()) > 0.8) {
        // 0.8 = 45 degree
        // todo: clear whole line or just to this segment?
        currentGuess_.erase(currentGuess_.begin() + i, currentGuess_.end());
//        ROS_INFO_STREAM("clipped line because of angle difference");
        done = true;
      } else if(currentGuess_.at(i).wP2_.x > maxViewRange_) {
//        ROS_INFO_STREAM("clipped line because of maxViewRange_");
        currentGuess_.erase(currentGuess_.begin() + i, currentGuess_.end());
        done = true;
      } else if((i < (currentGuess_.size() - 1)) && currentGuess_.at(i).getLength() > 0.3) {
        // if a segment is longer than 0.3m, the line is probably not dashed (so no middle line)
        // excluding the last segment, since we created this for searching forward
        ROS_INFO_STREAM("clipped line because of length");
        currentGuess_.erase(currentGuess_.begin() + i, currentGuess_.end());
        done = true;
      }
    }

    currentMiddleLine_.clear();
    currentMiddleLine_.insert(currentMiddleLine_.begin(), currentGuess_.begin(), currentGuess_.end() - (done ? 0 : 1));

#ifdef PUBLISH_DEBUG
    // draw our middle line
    for(auto l : currentMiddleLine_) {
      cv::line(outputImg, l.iP1_, l.iP2_, cv::Scalar(255,0,255), 2, cv::LINE_AA);
    }
#endif
  }

  // todo: move old guess with odometry and compare old and new guess

#ifdef PUBLISH_DEBUG
  debugImgPub_.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, outputImg).toImageMsg());
#endif

  // publish points to road topic
  drive_ros_msgs::RoadLine msgMidLine;
  msgMidLine.lineType = drive_ros_msgs::RoadLine::MIDDLE;
  for(auto l : currentMiddleLine_) {
    geometry_msgs::PointStamped pt1, pt2;
    pt1.point.x = l.wP1_.x;
    pt1.point.y = l.wP1_.y;
    msgMidLine.points.push_back(pt1);
    pt2.point.x = l.wP2_.x;
    pt2.point.y = l.wP2_.y;
    msgMidLine.points.push_back(pt2);
  }

  line_output_pub_.publish(msgMidLine);

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
