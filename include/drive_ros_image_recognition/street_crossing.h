#ifndef STREET_CROSSING_H
#define STREET_CROSSING_H

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <drive_ros_image_recognition/common_image_operations.h>

#include "drive_ros_image_recognition/RoadLane.h"

// todo: a lot of this stuff is used by new_road_detection too, so we should put this in a header / class
//typedef boost::shared_ptr<cv::Mat> CvImagePtr;

//inline CvImagePtr convertImageMessage(const sensor_msgs::ImageConstPtr& imageIn) {
//  CvImagePtr cvPtr;
//  try
//  {
//    // hardcopies for now, might be possible to process on pointer if fast enough
//    // todo: make prettier
//    cv_bridge::CvImagePtr tmpPtr = cv_bridge::toCvCopy(*imageIn, "");
//    cvPtr.reset(new cv::Mat(tmpPtr->image) );
//  }
//  catch(cv_bridge::Exception& e)
//  {
//    ROS_ERROR("cv_bridge exception: %s", e.what());
//    return NULL;
//  }

//  if(!cvPtr->data)
//  {
//    ROS_WARN("Empty image received, skipping!");
//    return NULL;
//  }
//  return cvPtr;
//}

namespace drive_ros_image_recognition {

class StreetCrossingDetection {

private:
    // configs
    int sobelThreshold;
    float minLineWidthMul;
    float maxLineWidthMul;
    float lineWidth; // todo: should be in config

    // variables
    std::vector<SearchLine> searchLines;
    CvImagePtr currentImage;
    CvImagePtr currentSobelImage;

    // communication
    image_transport::ImageTransport imageTransport;
    image_transport::Subscriber imageSubscriber;

//    ros::Publisher linePublisher;

    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    void imageCallback(const sensor_msgs::ImageConstPtr& imageIn);


#ifdef DRAW_DEUBG
    image_transport::Publisher debugImagePublisher;
    cv::Mat debugImage;
#endif

    // methods
    bool findStartline();
    std::vector<cv::Point2f> processSearchLine(SearchLine &sl);

    TransformHelper transform_helper_;
    ImageOperator image_operator_;

public:
    StreetCrossingDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh);
    ~StreetCrossingDetection();
    bool init();
};

} // namepsace drive_ros_image_recognition

#endif // STREET_CROSSING_H
