#ifndef ROAD_MODEL_H
#define ROAD_MODEL_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "drive_ros_image_recognition/polynom.hpp"
#include "drive_ros_image_recognition/line.h"
#include "drive_ros_image_recognition/common_image_operations.h"

namespace drive_ros_image_recognition {

enum SegmentType {
	NORMAL_ROAD, INTERSECTION_STOP, INTERSECTION_GO_STRAIGHT
};

bool transformOdomPointsToRearAxis(
		tf::TransformListener *pTfListener,
		std::vector<tf::Stamped<tf::Point>> &odomPts,
    std::vector<tf::Stamped<tf::Point>> &rearAxisPts);

bool transformRearAxisPointsToOdom(
		tf::TransformListener *pTfListener,
		std::vector<cv::Point2f> &rearAxisPts,
    std::vector<tf::Stamped<tf::Point>> &odomPts);

struct Segment {
    cv::Point2f positionWorld;
    cv::Point2f positionImage;
    cv::Point2f endPositionWorld, endPositionImage;
    cv::Point2f leftLineStart, leftLineEnd, rightLineStart, rightLineEnd;
    cv::Point2f leftPosW, midPosW, rightPosW; // These points lay on the left/mid/right line marking
    cv::Point2f leftPosI, midPosI, rightPosI;
    float angleDiff; // The angle difference to the previous segment
    float angleTotal; // The angle in which the lane is heading to
    float length; // Length of the segment in world coordinates [m]
    float probablity; // The probability of this segment
    SegmentType segmentType = SegmentType::NORMAL_ROAD;
    int ttl = 4;
    bool leftLineSet = false, rightLineSet = false;

    // Values for use with odometry
    tf::Stamped<tf::Point> odomPosition, odomStart, odomEnd;
    tf::Stamped<tf::Point> odomLeftLineStart, odomLeftLineEnd;
    tf::Stamped<tf::Point> odomRightLineStart, odomRightLineEnd;
    ros::Time creationTimestamp;
    bool odomPointsSet = false;

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

    inline bool isIntersection() const { return
    		segmentType == SegmentType::INTERSECTION_GO_STRAIGHT ||
			segmentType == SegmentType::INTERSECTION_STOP;
    }
};

class RoadModel {
	tf::TransformListener *pTfListener;

    Polynom currentDrivingLinePoly;
	int defaultPolyOrder = 2;
    float maxPolyError = 10.f;
	int noNewSegmentsCtr = 0; // DEBUG
    float laneWidth;
    bool driveStraight = false;

public:
    RoadModel(tf::TransformListener *tfListener, float laneWidth_)
		: pTfListener(tfListener)
        , laneWidth(laneWidth_)
	{
	}

    std::vector<Segment> segmentsToDl; // TODO: make private again

    inline void setLaneWidth(float w) { laneWidth = w; }
    inline void setDefaultPolyOrder(int o) { defaultPolyOrder = o; }
    inline void setMaxPolyErrorThresh(float t) { maxPolyError = t; }

    void getSegmentPositions(std::vector<cv::Point2f> &positions, std::vector<float> &angles, ros::Time stamp);
    void updateSegmentAtIndex(Segment &seg, int index);
    void setOdomPointsForSegments();
    void decreaseAllSegmentTtl();
    void decreaseSegmentTtl(int index);

    bool segmentFitsToPrevious(Segment *segmentToAdd, int index);
    float getDrivingLine(Polynom &drivingLine);
    bool getLaneMarkings(Polynom &line, bool leftLine);
};

} // namespace drive_ros_image_recognition

#endif // ROAD_MODEL_H
