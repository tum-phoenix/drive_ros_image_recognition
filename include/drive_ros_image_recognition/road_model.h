#ifndef ROAD_MODEL_H
#define ROAD_MODEL_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "drive_ros_image_recognition/polynom.hpp"
#include "drive_ros_image_recognition/line.h"
#include "drive_ros_image_recognition/common_image_operations.h"

namespace drive_ros_image_recognition {

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
    float angleTotal; // The angle in which the lane is heading to
    float length; // Length of the segment in world coordinates [m]
    float probablity; // The probability of this segment
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

    Segment(cv::Point2f worldPos, cv::Point2f imagePos, float angleTotal_, float len, float prob)
        : positionWorld(worldPos)
        , positionImage(imagePos)
        , angleTotal(angleTotal_)
        , length(len)
        , probablity(prob)
    {
    }

};

struct Intersection {
	float distanceTo;
	float confidence;
	bool stopLineFound;
	tf::Stamped<tf::Point> odomPosition;
	bool odomSet = false;

	Intersection(float dist, float conf, bool hasStopLine)
	: distanceTo(dist)
	, confidence(conf)
	, stopLineFound(hasStopLine)
	{
	}
};

class PolynomAverageFilter {
	std::vector<Polynom> polyHistory;
	std::vector<float> rangeHistory;
	bool historyFilled = false;
	int historyIdx = 0;
	int polyOrder;
	int historyLen;

public:
	PolynomAverageFilter(int polynomOrder, int historyLength)
	: polyOrder(polynomOrder)
	, historyLen(historyLength)
	{
	}

	float addPolynom(Polynom &out, Polynom &in, float detectionRange);
	void setPolyOrder(int o);
	void setHistoryLength(int l);
};

class RoadModel {
	tf::TransformListener *pTfListener;

    Polynom currentDrivingLinePoly;
    float currentDetectionRange = .0f;

	int defaultPolyOrder = 3;
    float maxPolyError = 10.f;
    float maxAngleDiff = .7f;
    float minSegmentProb = .1f;
    float laneWidth;

    PolynomAverageFilter midLineHistory = PolynomAverageFilter(defaultPolyOrder,5);
    std::vector<Intersection> intersections;
    std::vector<Segment> segmentsToDl;

    bool getLaneMarkings(Polynom &line, bool leftLine);

public:
    RoadModel(tf::TransformListener *tfListener, float laneWidth_)
		: pTfListener(tfListener)
        , laneWidth(laneWidth_)
	{
	}

    inline void setLaneWidth(float w) { laneWidth = w; }
    inline void setMaxPolyErrorThresh(float t) { maxPolyError = t; }
    inline void setMaxAngleDiff(float d) { maxAngleDiff = d; }
    inline void setMinSegmentProb(float p) { minSegmentProb = p; }

    void setDefaultPolyOrder(int o);
    void setPolyHistoryLen(int l);

    void getSegmentPositions(std::vector<cv::Point2f> &positions, std::vector<float> &angles, ros::Time stamp);
    void updateSegmentAtIndex(Segment &seg, int index);
    void setOdomPointsForSegments();
    void decreaseAllSegmentTtl();
    void decreaseSegmentTtl(int index);

    void addIntersectionAt(float x, float confidence, bool hasStopLine);
    bool getIntersections(std::vector<tf::Stamped<tf::Point>> &positions, std::vector<float> &confidences,
    		std::vector<bool> &hasStopLine, Polynom &drivingLine);

    bool segmentFitsToPrevious(Segment *segmentToAdd, int index, bool &possibleIntersection);
    void buildDrivingLine();
    float getDrivingLine(Polynom &drivingLine);
    inline bool getLeftLaneMarking(Polynom &line) { return getLaneMarkings(line, true); }
    inline bool getRightLaneMarking(Polynom &line) { return getLaneMarkings(line, false); }

};

} // namespace drive_ros_image_recognition

#endif // ROAD_MODEL_H
