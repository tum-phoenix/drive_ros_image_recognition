#ifndef ROAD_MODEL_H
#define ROAD_MODEL_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "drive_ros_image_recognition/polynom.hpp"
#include "drive_ros_image_recognition/line.h"
#include "drive_ros_image_recognition/common_image_operations.h"

#define PREDICTION_LENGTH   1.0

namespace drive_ros_image_recognition {

enum SegmentType {
	NORMAL_ROAD, INTERSECTION_STOP, INTERSECTION_GO_STRAIGHT
};

void transformOdomPointsToRearAxis(
		tf::TransformListener *pTfListener,
		std::vector<tf::Stamped<tf::Point>> &odomPts,
		std::vector<tf::Stamped<tf::Point>> &rearAxisPts,
		ros::Time stamp);

void transformRearAxisPointsToOdom(
		tf::TransformListener *pTfListener,
		std::vector<cv::Point2f> &rearAxisPts,
		std::vector<tf::Stamped<tf::Point>> &odomPts,
		ros::Time stamp);

struct Segment {
    cv::Point2f positionWorld;
    cv::Point2f positionImage;
    cv::Point2f leftPosW, midPosW, rightPosW; // These points lay on the left/mid/right line marking
    cv::Point2f leftPosI, midPosI, rightPosI;
    float angleDiff; // The angle difference to the previous segment
    float angleTotal; // The angle in which the lane is heading to
    float length; // This is defined globally and hence will be the same for every segment
    float probablity; // The probability of this segment
    SegmentType segmentType = SegmentType::NORMAL_ROAD;

    // Values for use with odometry
    tf::Stamped<tf::Point> odomPosition;
    ros::Time creationTimestamp;

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
    };
};

struct DrivingLane {
	ros::Time stamp;
	float detectionRange;
	Polynom poly;

	DrivingLane()
	: detectionRange(0.f)
	{
	}

	DrivingLane(Polynom &p, float pointXRange, ros::Time &timestamp)
	: poly(p)
	, detectionRange(pointXRange)
	, stamp(timestamp)
	{
	}
};

class RoadModel {
	tf::TransformListener *pTfListener;
	std::vector<Segment> drivingLine;
	DrivingLane dl;
	int noNewSegmentsCtr = 0; // DEBUG

public:
    RoadModel(tf::TransformListener *tfListener)
		: pTfListener(tfListener)
	{
	}

    // Poly based
    void addLanePoints(std::vector<cv::Point2f> &lanePoints, ros::Time stamp);

    DrivingLane getDrivingLine() { return dl; }

    // Segment based
    void addSegments(std::vector<Segment> &newSegments, ros::Time timestamp);
    void getSegmentSearchStart(cv::Point2f &posWorld, float &angle) const;
    bool segmentFitsToPrevious(Segment *previousSegment, Segment *segmentToAdd, bool isFirstSegment);
    std::vector<tf::Stamped<tf::Point>> getDrivingLinePts();
};

} // namespace drive_ros_image_recognition

#endif // ROAD_MODEL_H
