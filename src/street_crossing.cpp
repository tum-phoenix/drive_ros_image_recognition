#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
//#include "drive_ros_image_recognition/warp_image.h"
#include "drive_ros_image_recognition/street_crossing.h"

namespace drive_ros_image_recognition {
namespace detection {

StreetCrossingDetection::StreetCrossingDetection(const ros::NodeHandle nh_, const ros::NodeHandle pnh_)
  : nh(nh_)
  , pnh(pnh_)
  , imageTransport(nh_)
  , sobelThreshold(50)
  , minLineWidthMul(0.5)
  , maxLineWidthMul(2.0)
  , lineWidth(0.4)
  #ifdef DRAW_DEBUG
  , debugImage(0, 0, CV_8UC1)
  #endif
{
}

StreetCrossingDetection::~StreetCrossingDetection() {

}

bool StreetCrossingDetection::init() {
  imageSubscriber = imageTransport.subscribe("img_in", 10, &StreetCrossingDetection::imageCallback, this);
  // todo: advertise lines

#ifdef DRAW_DEBUG
  debugImagePublisher = imageTransport.advertise("/street_crossing/debug_image", 10);
#endif

  return true;
}

// todo:
// dynamic reconfigure

void StreetCrossingDetection::imageCallback(const sensor_msgs::ImageConstPtr &imgIn) {
  currentImage = convertImageMessage(imgIn);
  findStartline();
}

bool StreetCrossingDetection::findStartline() {
  std::vector<cv::Point2f> linePoints;
  SearchLine mySl;

  // Sobel
  currentSobelImage.reset(new cv::Mat(currentImage->rows, currentImage->cols, CV_8UC1));
  cv::Sobel(*currentImage, *currentSobelImage, -1, 0, 1);

  // use search-line to find stop-line
  mySl.iStart = cv::Point2f(currentImage->cols / 2, currentImage->rows * .8);
  mySl.iEnd = cv::Point2f(currentImage->cols / 2, currentImage->rows * .4);
  // imageToWorld(mySl.iStart, mySl.wStart);
  // imageToWorld(mySl.iEnd, mySl.wEnd);
  linePoints = processSearchLine(mySl);


  // Draw / publish points
#ifdef DRAW_DEBUG
  //  debugImage = currentSobelImage->clone();
  debugImage = currentImage->clone();
  // Image dimensions:1280x344
  for(auto point : linePoints) {
    cv::line(debugImage, cv::Point(point.x - 40, point.y), cv::Point(point.x + 40, point.y), cv::Scalar(255));
  }
  // draw searchline
  cv::line(debugImage, mySl.iStart, mySl.iEnd, cv::Scalar(255));
  debugImagePublisher.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_8UC1, debugImage).toImageMsg());
#endif
}

std::vector<cv::Point2f> StreetCrossingDetection::processSearchLine(SearchLine &sl) {
    std::vector<cv::Point2f> foundPoints;
    bool foundLowHigh = false;
    int pxlCounter = 0;
    cv::Point2f iStartPxlPeak;

    cv::LineIterator lineIt(*currentSobelImage, sl.iStart, sl.iEnd);
    cv::Rect rect(0, 0, currentSobelImage->cols, currentSobelImage->rows);
    //  float iDist = cv::norm(sl.iEnd - sl.iStart);
    //  float wDist = cv::norm(sl.wEnd - sl.wStart);

    for(int i = 0; i < lineIt.count; i++, ++lineIt) {
        // safety check : is the point inside the image
        if(!rect.contains(lineIt.pos())){

            ROS_WARN("Received an invalid point outside the image to check for Sobel (%i,%i)", lineIt.pos().x, lineIt.pos().y);
            return foundPoints;
        }

        // Search for a bright pixel. Searching from bottom to top
        int sobel = currentSobelImage->at<char>(lineIt.pos());

        if(sobel > sobelThreshold) {
            // found low-high pass
            if(!foundLowHigh) {
                foundLowHigh = true;
                pxlCounter = 0;
                iStartPxlPeak.x = lineIt.pos().x;
                iStartPxlPeak.y = lineIt.pos().y;
            }
        } else if(sobel < -sobelThreshold) {
            // found high-low pass
            // check if we found a lowHigh + highLow border
            if(foundLowHigh){
                //check if the points have the right distance
                // TODO to bad, calculate for each road line (how should we use them for searching?
                // float pxlPeakWidth = iDist / wDist * lineWidth; // todo: understand this

                //logger.debug("")<<"crossing found highLow: "<<pxlCounter<<" "<<pxlPeakWidth;
                //logger.debug("")<<"crossing found max: "<<pxlPeakWidth*maxLineWidthMul;
                //logger.debug("")<<"crossing found min: "<<pxlPeakWidth*minLineWidthMul;

                // todo: check if the line we found has the correct height
                // if((pxlCounter > (pxlPeakWidth * minLineWidthMul)) && (pxlCounter < (pxlPeakWidth * maxLineWidthMul))) {
                // return the middle of the line we found
                // cv::Point2f iMid((iStartPxlPeak.x + lineIt.pos().x) *.5, (iStartPxlPeak.x + lineIt.pos().y) * .5);

                foundPoints.push_back(lineIt.pos());
                // cv::Point2f wMid;
                // imageToWorld(iMid, wMid);
                // foundPoints.push_back(wMid);
                // ROS_DEBUG("Found a stop-line");
                // }
            }
            pxlCounter = 0;
            foundLowHigh = false;
            // if not, we dont have to do anything
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
