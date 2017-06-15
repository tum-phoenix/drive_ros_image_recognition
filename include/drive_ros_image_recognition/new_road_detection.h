#ifndef NEW_ROAD_DETECTION_H
#define NEW_ROAD_DETECTION_H

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <drive_ros_image_recognition/RoadLane.h>
//#include <lms/module.h>
//#include <street_environment/road.h>
//#include <street_environment/car.h>
//#include <lms/imaging/transform_image.h>
//#include <lms/imaging/image.h>

#include <list>

// for multithreading
#include <mutex>
#include <condition_variable>
#include <thread>

typedef std::shared_ptr<cv::Mat> CvImagePtr;

inline CvImagePtr convertImageMessage(const sensor_msgs::ImageConstPtr& img_in) {
  CvImagePtr cv_ptr;
  try
  {
    // todo: check if this if we have 8 or 16 greyscale images
    // hardcopies for now, might be possible to process on pointer if fast enough
    cv_ptr = cv_bridge::toCvCopy(msg, "mono16");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if( !cv_ptr->data)
  {
    ROS_WARN("Empty image received, skipping!");
    return;
  }
  return cv_ptr;
}

/**
 * @brief Port of LMS module new_road_detection to ROS
 **/
class NewRoadDetection {
    // configs
    float searchOffset_;
    float distanceBetweenSearchlines_;
    bool findPointsBySobel_;
    bool renderDebugImage_;
    float minLineWidthMul_;
    float maxLineWidthMul_;
    int threshold_;
    float laneWidthOffsetInMeter_;
    bool translate_environment_;
    bool useWeights_;
    int sobelThreshold_;
    int numThreads_; // 0 means single threaded

    // todo: port this
//    lms::imaging::Homography homo;
    //Datachannels
    // todo: we have not determined all interfaces yet, so will leave this in for now until we have figured it out
//    lms::ReadDataChannel<lms::imaging::Image> image;
//    lms::WriteDataChannel<street_environment::RoadLane> road;
//    lms::WriteDataChannel<street_environment::RoadLane> output;
//    lms::WriteDataChannel<lms::imaging::Image> debugImage;
//    lms::WriteDataChannel<lms::math::polyLine2f> debugAllPoints;
//    lms::WriteDataChannel<lms::math::polyLine2f> debugValidPoints;
//    lms::WriteDataChannel<lms::math::polyLine2f> debugTranslatedPoints;
//    lms::ReadDataChannel<street_environment::CarCommand> car;

    struct SearchLine{
        cv::Point2f w_start;
        cv::Point2f w_end;
        cv::Point i_start;
        cv::Point i_end;


        cv::Point2f w_left;
        cv::Point2f w_mid;
        cv::Point2f w_right;
    };

    std::list<SearchLine> lines;

    std::mutex mutex;
    std::mutex debugAllPointsMutex;
    std::mutex debugValidPointsMutex;
    std::vector<std::thread> threads;
    std::condition_variable conditionNewLine;
    std::condition_variable conditionLineProcessed;
    bool threadsRunning;
    int linesToProcess;

    ros::Subscriber image_sub_;
    // road inputs and outputs
    ros::Subscriber road_;
    ros::Publisher output_;
#ifdef DRAW_DEBUG
    ros::Publisher debug_img_pub_;
#endif
    ros::NodeHandle nh_;
    std::string my_namespace_;

public:
    bool init() override;
    void destroy() override;
    void imageCallback(const sensor_msgs::ImageConstPtr img_in);
    void configsChanged() override;

    bool find();

    std::vector<lms::math::vertex2f>
    findBySobel(const bool renderDebugImage,
                const std::vector<int> &xv,
                const std::vector<int> &yv,
                const float minLineWidthMul,
                const float maxLineWidthMul,
                const float iDist,
                const float wDist,
                const int threshold);

    std::vector<lms::math::vertex2f>
    findByBrightness(const bool renderDebugImage,
                     const std::vector<int> &xv,
                     const std::vector<int> &yv,
                     const float minLineWidthMul,
                     const float maxLineWidthMul,
                     const float iDist,
                     const float wDist,
                     const int threshold);

    void processSearchLine(const SearchLine &line);

    void threadFunction();
};

#endif // NEW_ROAD_DETECTION_H
