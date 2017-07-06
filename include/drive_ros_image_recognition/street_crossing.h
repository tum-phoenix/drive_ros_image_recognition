#ifndef STREET_CROSSING_H
#define STREET_CROSSING_H

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "geometry_common.h"
#include "drive_ros_image_recognition/RoadLane.h"

//#define DRAW_DEBUG

typedef boost::shared_ptr<cv::Mat> CvImagePtr;

inline CvImagePtr convertImageMessage(const sensor_msgs::ImageConstPtr& imageIn) {
  CvImagePtr cvPtr;
  try
  {
    // hardcopies for now, might be possible to process on pointer if fast enough
    // todo: make prettier
    cv_bridge::CvImagePtr tmpPtr = cv_bridge::toCvCopy(*imageIn, "");
    cvPtr.reset(new cv::Mat(tmpPtr->image) );
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return NULL;
  }

  if(!cvPtr->data)
  {
    ROS_WARN("Empty image received, skipping!");
    return NULL;
  }
  return cvPtr;
}

namespace drive_ros_image_recognition {
namespace detection {

class StreetCrossingDetection {

private:
    struct SearchLine{
        cv::Point2f wStart;
        cv::Point2f wEnd;
        cv::Point2i iStart;
        cv::Point2i iEnd;
    };

    // configs
    int sobelThreshold;
    float minLineWidthMul;
    float maxLineWidthMul;
    float lineWidth; // in m; todo: should be in config

    // variables
    std::vector<SearchLine> searchLines;
    CvImagePtr currentImage;

    // communication
    image_transport::ImageTransport imageTransport;
    image_transport::Subscriber imageSubscriber;
    // road inputs and outputs
    ros::Subscriber roadSubscriber;
    ros::Publisher linePublisher;

    drive_ros_image_recognition::RoadLane roadBuffer;

    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    void roadCallback(const drive_ros_image_recognition::RoadLaneConstPtr& roadIn);
    void imageCallback(const sensor_msgs::ImageConstPtr& imageIn);

#ifdef DRAW_DEBUG
    image_transport::Publisher debugImagePublisher;
    cv::Mat debugImage;
//    image_transport::Publisher detectedPointsPublisher;
#endif

    ros::ServiceClient worldToImageClient;
    ros::ServiceClient imageToWorldClient;
    bool imageToWorld(const cv::Point &imagePoint, cv::Point2f &worldPoint);
    bool worldToImage(const cv::Point2f &worldPoint, cv::Point &imagePoint);

    // methods
    bool findStartline(/*linestring &middleLine*/);
    std::vector<cv::Point2f> processSearchLine(SearchLine &sl);

public:
    StreetCrossingDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh);
    ~StreetCrossingDetection();
    bool init();
};

} // namepsace drive_ros_image_recognition
} // namespace detection

#endif // STREET_CROSSING_H
