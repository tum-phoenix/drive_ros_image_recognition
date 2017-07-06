#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include "drive_ros_image_recognition/warp_image.h"
#include "drive_ros_image_recognition/street_crossing.h"

namespace drive_ros_image_recognition {
namespace detection {

StreetCrossingDetection::StreetCrossingDetection(const ros::NodeHandle nh_, const ros::NodeHandle pnh_)
  : nh(nh_) // help: difference nh und pnh (whats the input)
  , pnh(pnh_)
  , imageTransport(nh_)
  , sobelThreshold(50)
  , minLineWidthMul(0.5)
  , maxLineWidthMul(2.0)
  , lineWidth(0.4)
#ifdef DRAW_DEBUG
  , debugImage(0, 0, CV_16S)
#endif
{
  // todo
}

StreetCrossingDetection::~StreetCrossingDetection() {

}

bool StreetCrossingDetection::init() {
  imageSubscriber = imageTransport.subscribe("img_in", 10, &StreetCrossingDetection::imageCallback, this);
  //roadSubscriber = pnh.subscribe("road_in", 100, &StreetCrossingDetection::roadCallback, this);
  // todo: advertise lines

  // help: serviceClients
//  imageToWorldClient = nh.serviceClient<drive_ros_image_recognition::ImageToWorld>("/ImageToWorld");
//  worldToImageClient = nh.serviceClient<drive_ros_image_recognition::WorldToImage>("/WorldToImage");

  // todo: debug channels
  // help: how to publish debug picture / points?
#ifdef DRAW_DEBUG
  debugImagePublisher = imageTransport.advertise("/street_crossing/debug_image", 10);
#endif

  return true;
}

//bool StreetCrossingDetection::imageToWorld(const cv::Point &image_point, cv::Point2f &world_point) {
//  drive_ros_image_recognition::ImageToWorld srv;
//  geometry_msgs::Point image_point_send;
//  image_point_send.x = image_point.x;
//  image_point_send.y = image_point.y;
//  image_point_send.z = 0.0;
//  srv.request.image_point = image_point_send;
//  if (imageToWorldClient.call(srv))
//  {
//    world_point.x = srv.response.world_point.point.x;
//    world_point.y = srv.response.world_point.point.y;
//    return true;
//  }
//  else
//  {
//    ROS_ERROR("Failed to call service ImageToWorld");
//    return false;
//  }
//}

//bool StreetCrossingDetection::worldToImage(const cv::Point2f &world_point, cv::Point &image_point) {
//  drive_ros_image_recognition::WorldToImage srv;
//  geometry_msgs::PointStamped world_point_send;
//  world_point_send.point.x = world_point.x;
//  world_point_send.point.y = world_point.y;
//  world_point_send.point.z = 0.0;
//  // todo: check if this is valid, we assume the trajectory is defined in the axis reference frame
//  world_point_send.header.frame_id = "tf_front_axis_middle";
//  srv.request.world_point = world_point_send;
//  if (worldToImageClient.call(srv))
//  {
//    image_point.x = (int)srv.response.image_point.x;
//    image_point.y = (int)srv.response.image_point.y;
//    return true;
//  }
//  else
//  {
//    ROS_ERROR("Failed to call service WorldToImage");
//    return false;
//  }
//}

// todo:
// dynamic reconfigure

void StreetCrossingDetection::roadCallback(const drive_ros_image_recognition::RoadLaneConstPtr &roadIn) {
  roadBuffer = *roadIn;
}

void StreetCrossingDetection::imageCallback(const sensor_msgs::ImageConstPtr &imgIn) {
  currentImage = convertImageMessage(imgIn);
#ifdef DRAW_DEBUG
  debugImage = currentImage->clone();
#endif
//  if(roadBuffer.points.size() > 1) {
//    findStartline();
//  } else {
//    ROS_WARN("Not searching for start line, since road-buffer has no points stored");
//  }
  findStartline();
}

bool StreetCrossingDetection::findStartline(/*linestring& middleLine*/) {
    float hightStartLine = 0.04;
//    bool foundStartLine = false;
    linestring rightMiddleLine, middleLine, tmp;

    searchLines.clear();

    // Get middle line with distance and move to right
//    for(auto point : roadBuffer.points) {
//      boost::geometry::append(middleLine, point_xy(point.x, point.y));
//    }
//    drive_ros_geometry_common::moveOrthogonal(middleLine, tmp, lineWidth / 2);
//    drive_ros_geometry_common::simplify(tmp, rightMiddleLine, hightStartLine + 0.01);

    //get all lines
//    for(int i = 1; i < rightMiddleLine.size(); i++) {
//      SearchLine l;
//      // todo: if this works, make it more efficient
//      l.wStart = cv::Point2f(rightMiddleLine.at(i - 1).x(), rightMiddleLine.at(i - 1).y());
//      l.wEnd = cv::Point2f(rightMiddleLine.at(i).x(), rightMiddleLine.at(i).y());

////      cv::Point l_w_start, l_w_end;
////      worldToImage(l.w_start, l_w_start);
////      worldToImage(l.w_end, l_w_end);
//      worldToImage(l.wStart, l.iStart);
//      worldToImage(l.wEnd, l.iEnd);
//      cv::Rect imgRect(cv::Point(),cv::Point(currentImage->cols, currentImage->rows));
////      if(!imgRect.contains(l_w_start)){
////        // try middle lane -> should always be in image
////        l.w_start = cv::Point2f(bg::get<0>(*it_mid),bg::get<1>(*it_mid));
////        worldToImage(l.w_start, l_w_start);
////        if(imgRect.contains(l_w_start)){
////          continue;
////        }
////      }else if(!imgRect.contains(l_w_end)){
////        l.w_end = cv::Point2f(bg::get<0>(*it_mid),bg::get<1>(*it_mid));
////      }

//      //transform them in image-coordinates
////      std::unique_lock<std::mutex> lock(mutex);
////      worldToImage(l.w_start,l.i_start);
////      worldToImage(l.w_end,l.i_end);
////      lines_.push_back(l);
////      linesToProcess_++;

////      if((imgRect.contains(l.iStart)) && (imgRect.contains(l.iEnd))) {
////        searchLines.push_back(l);
////      }

//      // todo: figure out what this does (no internet)
//      //        conditionNewLine.notify_one();
//    }

//    ROS_INFO("Number of search-lines to search stop-line: %lu", searchLines.size());

    // Sobel
    cv::Mat /*imgGaussian,*/ imgSobel, absGradiants;
    //    cv::GaussianBlur(*currentImage, imgGaussian, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);
        cv::Sobel(*currentImage, imgSobel, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);
//        cv::convertScaleAbs(imgSobel, absGradiants);
        cv::convertScaleAbs(imgSobel, debugImage);

        // use search-lines to find stop-line
        // todo: if this works, parallize it!
        std::vector<cv::Point2f> linePoints;
    //    for(auto sl : searchLines) {
    //      auto tmp = processSearchLine(sl);
    //      linePoints.insert(linePoints.end(), tmp.begin(), tmp.end());
    //    }

        SearchLine mySl;
        mySl.iStart = cv::Point2f(currentImage->cols / 2, currentImage->rows * 0.5);
        mySl.iEnd = cv::Point2f(currentImage->cols / 2, currentImage->rows * .75);

//        imageToWorld(mySl.iStart, mySl.wStart);
//        imageToWorld(mySl.iEnd, mySl.wEnd);
        linePoints = processSearchLine(mySl);

        // Draw / publish points
    #ifdef DRAW_DEBUG
        for(auto point : linePoints) {
          ROS_INFO("drawing line-point");
          cv::circle(debugImage, point, 2, cv::Scalar(255, 0, 0));
        }
//        cv::circle(debugImage, mySl.iStart, 2, cv::Scalar(0, 255, 0));
//        cv::circle(debugImage, mySl.iEnd, 2, cv::Scalar(0, 255, 0))
        cv::line(debugImage, mySl.iStart, mySl.iEnd, cv::Scalar(255));
        debugImagePublisher.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_8UC1, debugImage).toImageMsg());
    #endif

    }

    std::vector<cv::Point2f> StreetCrossingDetection::processSearchLine(SearchLine &sl) {
      std::vector<cv::Point2f> foundPoints;
  bool foundLowHigh = false;
  int pxlCounter = 0;
  cv::Point2f iStartPxlPeak;
  cv::LineIterator lineIt(*currentImage, sl.iStart, sl.iEnd);
  ROS_INFO("processSearchLine");

  float iDist = cv::norm(sl.iEnd - sl.iStart);
//  float wDist = cv::norm(sl.wEnd - sl.wStart);
  cv::Rect rect(cv::Point(),cv::Point(currentImage->rows, currentImage->cols));

  for(int i = 0; i < lineIt.count; i++, ++lineIt) {
    // safety check : is the point inside the image
    if(!rect.contains(lineIt.pos())){
      ROS_WARN("StreetCrossing::processSearchLine: Received an invalid point outside the image to check for Sobel");
      return foundPoints;
    }

    // Search for a bright pixel. Searching from bottom to top
    int sobel = (int)currentImage->at<unsigned char>(lineIt.pos().x, lineIt.pos().y);
    if(sobel > sobelThreshold) {
      // found low-high pass
      if(!foundLowHigh) {
        foundLowHigh = true;
        pxlCounter = 0;
        iStartPxlPeak.x = lineIt.pos().x;
        iStartPxlPeak.y = lineIt.pos().y;
      }
    } else if(sobel < -sobelThreshold){
      // found high-low pass
      //check if we found a lowHigh + highLow border
      if(foundLowHigh){
        //check if the points have the right distance
        // TODO to bad, calculate for each road line (how should we use them for searching?
//        float pxlPeakWidth = iDist / wDist * lineWidth; //todo: understand this

        //logger.debug("")<<"crossing found highLow: "<<pxlCounter<<" "<<pxlPeakWidth;
        //logger.debug("")<<"crossing found max: "<<pxlPeakWidth*maxLineWidthMul;
        //logger.debug("")<<"crossing found min: "<<pxlPeakWidth*minLineWidthMul;

        // returns the middle of a line from multiple points
//        if((pxlCounter > (pxlPeakWidth * minLineWidthMul)) && (pxlCounter < (pxlPeakWidth * maxLineWidthMul))) {
          // we found a valid point, get the middle
          cv::Point2f iMid = (iStartPxlPeak + cv::Point2f(lineIt.pos().x, lineIt.pos().y)) * .5;
//          cv::Point2f wMid;
//          imageToWorld(iMid, wMid);
//          foundPoints.push_back(wMid);
//          ROS_DEBUG("Found a stop-line");
          //logger.debug("")<<"crossing FOUND VALID CROSSING";
//        }
      }
      pxlCounter = 0;
      foundLowHigh = false;
      //if not, we dont have to do anything
    }

    // for calculation of line width
    if(foundLowHigh){
      pxlCounter++;
    }
  }
  return foundPoints;
}

} //namepsace detection
} //namespace drive_ros_image_recognition
