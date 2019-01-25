#include <ros/ros.h>
#include "drive_ros_image_recognition/road_model.h"

namespace drive_ros_image_recognition {


bool transformOdomToRearAxis(
		tf::TransformListener *pTfListener,
		tf::Stamped<tf::Point> &odomPt,
		tf::Stamped<tf::Point> &rearAxisPt)
{

	try {
		pTfListener->transformPoint("/rear_axis_middle_ground", ros::Time(0), odomPt, "/odom", rearAxisPt);
		//("/rear_axis_middle_ground", odomPt, rearAxisPt);
	} catch (tf::TransformException &ex){
		ROS_ERROR("transformOdomToRearAxis: %s",ex.what());
		return false;
	}
	return true;
}

bool transformOdomPointsToRearAxis(
		tf::TransformListener *pTfListener,
		std::vector<tf::Stamped<tf::Point>> &odomPts,
		std::vector<tf::Stamped<tf::Point>> &rearAxisPts,
		ros::Time stamp)
{

	rearAxisPts.resize(odomPts.size());

	for(int i = 0; i < odomPts.size(); i++) {
		if(!transformOdomToRearAxis(pTfListener, odomPts.at(i), rearAxisPts.at(i)))
			return false;

//		if(pTfListener->waitForTransform("/rear_axis_middle_ground", "/odom", stamp, ros::Duration(0.1))) {
//			try {
//				pTfListener->transformPoint("/rear_axis_middle_ground", odomPts.at(i), rearAxisPts.at(i));
//			} catch (tf::TransformException &ex) {
//				ROS_ERROR("%s",ex.what());
//				continue;
//			}
//		} else {
//			ROS_ERROR("waitForTransform timed out in RoadModel::addSegments");
//			return false;
//		}
	}

	return true;
}

bool transformRearAxisToOdom(
		tf::TransformListener *pTfListener,
		tf::Stamped<tf::Point> &rearAxisPt,
		tf::Stamped<tf::Point> &odomPt)
{
	try {
		pTfListener->transformPoint("/odom", rearAxisPt, odomPt);
	} catch (tf::TransformException &ex){
		ROS_ERROR("transformRearAxisToOdom: %s",ex.what());
		return false;
	}
	return true;
}

bool transformRearAxisPointsToOdom(
		tf::TransformListener *pTfListener,
		std::vector<cv::Point2f> &rearAxisPts,
		std::vector<tf::Stamped<tf::Point>> &odomPts,
		ros::Time stamp)
{
	odomPts.resize(rearAxisPts.size());

	for(int i = 0; i < rearAxisPts.size(); i++) {
		tf::Stamped<tf::Point> stampedRearAxis;
		stampedRearAxis.frame_id_ = "rear_axis_middle_ground";
		stampedRearAxis.stamp_ = ros::Time(0);
		stampedRearAxis.setX(rearAxisPts.at(i).x);
		stampedRearAxis.setY(rearAxisPts.at(i).y);
		stampedRearAxis.setZ(0.0f);

		if(!transformRearAxisToOdom(pTfListener, stampedRearAxis, odomPts.at(i))) {
			return false;
			ROS_ERROR("transformRearAxisPointsToOdom");
		}
	}

	return true;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * 					ROAD MODEL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void RoadModel::getSegmentPositions(std::vector<cv::Point2f> &positions, std::vector<float> &angles, ros::Time stamp) {
	std::vector<tf::Stamped<tf::Point>> odomPts, rearAxisPts;

	for(auto s : segmentsToDl) {
		if(s.odomPointsSet) {
			if(s.ttl > 0) {
				odomPts.push_back(s.odomStart);
				odomPts.push_back(s.odomEnd);
			} else {
				break;
			}
		} else {
			ROS_WARN("odom pts not set");
		}
	}

	transformOdomPointsToRearAxis(pTfListener, odomPts, rearAxisPts, stamp);

	positions.clear();
	angles.clear();

	for(int i = 0; i < rearAxisPts.size(); i += 2) {
		cv::Point2f startPt(rearAxisPts.at(i).x(),   rearAxisPts.at(i).y());
		cv::Point2f endPt  (rearAxisPts.at(i+1).x(), rearAxisPts.at(i+1).y());

//		ROS_INFO("startPt (%.2f, %.2f)", startPt.x, startPt.y);
//		ROS_INFO("  endPt (%.2f, %.2f)", endPt.x, endPt.y);
//		ROS_INFO(" angle = %.2f", atan2(endPt.y - startPt.y, endPt.x - startPt.x));

		if(startPt.x < 0.2f) {
			ROS_INFO("  remove first point");
		} else {
			positions.push_back(startPt);
			angles.push_back(atan2(endPt.y - startPt.y, endPt.x - startPt.x));
		}
	}

	ROS_INFO("getSegmentPositions: Returning %lu positions from %lu segments", positions.size(), odomPts.size() / 2);

}

void RoadModel::setOdomPointsForSegments() {
	ROS_INFO("setOdomPointsForSegments");
	std::vector<cv::Point2f> rearAxisPts;
	std::vector<tf::Stamped<tf::Point>> odomPts;

	for(int i = 0; i < segmentsToDl.size(); i++) {
		if(segmentsToDl.at(i).odomPointsSet) {
			continue;
		}

		rearAxisPts.clear();
		odomPts.clear();

		rearAxisPts.push_back(segmentsToDl.at(i).positionWorld);
		rearAxisPts.push_back(segmentsToDl.at(i).endPositionWorld);

		if(transformRearAxisPointsToOdom(pTfListener, rearAxisPts, odomPts, segmentsToDl.at(i).creationTimestamp)) {
			segmentsToDl.at(i).odomStart = odomPts.at(0);
			segmentsToDl.at(i).odomEnd = odomPts.at(1);
			segmentsToDl.at(i).creationTimestamp = odomPts.at(0).stamp_;
			segmentsToDl.at(i).odomPointsSet = true;
		}
	}
}

void RoadModel::updateSegmentAtIndex(Segment &seg, int index) {
	ROS_INFO("updateSegmentAtIndex %u", index);

	if(index < segmentsToDl.size()) {
		segmentsToDl.at(index) = seg;
	} else if(index == segmentsToDl.size()) {
		segmentsToDl.push_back(seg);
	} else {
		ROS_WARN("  index too high");
	}
}

bool RoadModel::segmentFitsToPrevious(Segment *segmentToAdd, int index) {
	ROS_INFO("segmentFitsToPrevious: segmentsToDl.size() = %lu, index = %u", segmentsToDl.size(), index);

    bool isFirstSegment = (index == 0);

	if((!isFirstSegment) && (index > segmentsToDl.size())) {
		return false;
	}

	Segment *previousSegment = (isFirstSegment ? nullptr : &(segmentsToDl.at(index - 1)));

	if(segmentToAdd->probablity < 0.2f) {
//		ROS_INFO("Segments probability is too low");
		return false;
	}

	if(segmentToAdd->length < 0.05f) {
		return false;
	}

	if(!isFirstSegment) {
		float angleVar = M_PI / 7.0f; // TODO move to config
		if(std::abs(previousSegment->angleTotal - segmentToAdd->angleTotal) > angleVar) {
//			ROS_INFO_STREAM("Too much angle difference: " << (segmentToAdd->angleDiff * 180.f / M_PI) << "[deg]");
			return false;
		}
	} else {
		// The first segment should points into the cars driving direction
		if(std::abs(segmentToAdd->angleTotal) > M_PI / 4.0f) {
//			ROS_WARN("First segments angle is too big: %.1f[deg]", segmentToAdd->angleTotal * 180.f / M_PI);
			return false;
		}
	}

    // Compare to previous polynom
    // TODO: quick hack. think this through
    float error = .0f;
    error += std::fabs(segmentToAdd->positionWorld.y - currentDrivingLinePoly.atX(segmentToAdd->positionWorld.x));
    error += std::fabs(segmentToAdd->endPositionWorld.y - currentDrivingLinePoly.atX(segmentToAdd->endPositionWorld.x));

    ROS_INFO("  Error based on polynom: %.3f", error);

    if(error > maxPolyError) {
        return false;
    }

	return true;
}

Polynom RoadModel::getDrivingLinePts(float &detectionRange) {
	std::vector<tf::Stamped<tf::Point>> odomPts, rearAxisPts;

	for(auto s : segmentsToDl) {
		if(s.odomPointsSet) {
			ROS_INFO("  Segment.ttl: %i", s.ttl);
			if(s.ttl > 0) {
				odomPts.push_back(s.odomStart);
				// Add a point in the middle of the segment
				auto odomMid = s.odomStart;
				odomMid.setX(odomMid.x() + .5f * (s.odomEnd.x() - s.odomStart.x()));
				odomMid.setY(odomMid.y() + .5f * (s.odomEnd.y() - s.odomStart.y()));
				odomPts.push_back(odomMid);
//				odomPts.push_back(s.odomEnd);
			} else {
				break;
			}
		} else {
			ROS_WARN("odom pts not set");
		}
	}

    transformOdomPointsToRearAxis(pTfListener, odomPts, rearAxisPts, ros::Time(0)); // TODO: time not used anymore

	std::vector<cv::Point2f> drivingLinePts;
	drivingLinePts.push_back(cv::Point2f(.3f, 0.f));
	for(auto sp : rearAxisPts) {
		drivingLinePts.push_back(cv::Point2f(sp.x(), sp.y()));
	}

	ROS_INFO("Built polynom from %lu points from %lu segments", drivingLinePts.size(), segmentsToDl.size());

    if(drivingLinePts.size() < 2) { // we always add a point in front of the vehicle
		// return a straight line
//        drivingLinePts.push_back(cv::Point2f(.3f, 0.f));
		drivingLinePts.push_back(cv::Point(.6, 0.f));
	}
    detectionRange = drivingLinePts.back().x;
	int polyOrder = (drivingLinePts.size() > 2) ? defaultPolyOrder : 1;
    currentDrivingLinePoly = Polynom(polyOrder, drivingLinePts);
    return currentDrivingLinePoly;
}

void RoadModel::decreaseAllSegmentTtl() {
	for(auto it = segmentsToDl.begin(); it != segmentsToDl.end(); ++it) {
		it->ttl = it->ttl - 1;
	}
}

void RoadModel::decreaseSegmentTtl(int index) {
	if(index < segmentsToDl.size()) {
		segmentsToDl.at(index).ttl = segmentsToDl.at(index).ttl - 1;
	}
}

} // namespace drive_ros_image_recognition
