#ifndef ROAD_MODEL_H
#define ROAD_MODEL_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "drive_ros_image_recognition/line.h"
#include "drive_ros_image_recognition/common_image_operations.h"

#define PREDICTION_LENGTH   1.0

namespace drive_ros_image_recognition {

enum SegmentType {
	NORMAL_ROAD, INTERSECTION_STOP, INTERSECTION_GO_STRAIGHT
};

struct Segment {
    cv::Point2f positionWorld;
    cv::Point2f positionImage;
    cv::Point2f leftPosW, midPosW, rightPosW; // These points lay on the left/mid/right line marking
    cv::Point2f leftPosI, midPosI, rightPosI;
    float angleDiff; // The angle difference to the previous segment
    float angleTotal; // The angle in which the lane is heading to
    float length; // This is defined globally and hence will be the same for every segment
    float probablity; // Not sure if we use this, yet
    SegmentType segmentType = SegmentType::NORMAL_ROAD;

    Segment()
    : probablity(0.f)
    {
    }

    Segment(cv::Point2f worldPos, cv::Point2f imagePos, float angleDiff_, float angleTotal_, float len, float prob)
        : positionWorld(worldPos)
        , positionImage(imagePos)
        , angleDiff(angleDiff_)
        , angleTotal(angleTotal_)
        , length(len)
        , probablity(prob)
    {
    }

    inline bool isIntersection() const { return segmentType == SegmentType::INTERSECTION_GO_STRAIGHT ||
        											segmentType == SegmentType::INTERSECTION_STOP; };
};

class RoadModel {
    std::vector<Segment> segments;

public:
    void addSegments(std::vector<Segment> &newSegments);
    void getFirstPosW(cv::Point2f &posW, float &angle) const;
};

} // namespace drive_ros_image_recognition

#endif // ROAD_MODEL_H
