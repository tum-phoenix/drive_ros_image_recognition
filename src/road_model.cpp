#include <ros/ros.h>
#include "drive_ros_image_recognition/road_model.h"

namespace drive_ros_image_recognition {

void transformOdomPointsToRearAxis(
		tf::TransformListener *pTfListener,
		std::vector<tf::Stamped<tf::Point>> &odomPts,
		std::vector<tf::Stamped<tf::Point>> &rearAxisPts,
		ros::Time stamp)
{

	for(int i = 0; i < odomPts.size(); i++) {

		if(pTfListener->waitForTransform("/rear_axis_middle_ground", "/odom", stamp, ros::Duration(0.1))) {
			try {
				pTfListener->transformPoint("/rear_axis_middle_ground", odomPts.at(i), rearAxisPts.at(i));
			} catch (tf::TransformException &ex) {
				ROS_ERROR("%s",ex.what());
				continue;
			}
		} else {
			ROS_ERROR("waitForTransform timed out in RoadModel::addSegments");
			break;
		}
	}
}

void transformRearAxisPointsToOdom(
		tf::TransformListener *pTfListener,
		std::vector<cv::Point2f> &rearAxisPts,
		std::vector<tf::Stamped<tf::Point>> &odomPts,
		ros::Time stamp)
{

	for(int i = 0; i < rearAxisPts.size(); i++) {
		tf::Stamped<tf::Point> stampedRearAxis;
		stampedRearAxis.frame_id_ = "/rear_axis_middle_ground";
		stampedRearAxis.stamp_ = stamp;
		stampedRearAxis.setX(rearAxisPts.at(i).x);
		stampedRearAxis.setY(rearAxisPts.at(i).y);
		stampedRearAxis.setZ(0.0f);

		// target_frame	The frame into which to transform
		// source_frame	The frame from which to transform
		// time	The time at which to transform
		// timeout	How long to block before failing
		if(pTfListener->waitForTransform("/odom", "/rear_axis_middle_ground", stamp, ros::Duration(0.1))) {
			try {
				pTfListener->transformPoint("/odom", stampedRearAxis, odomPts.at(i));

			}
			catch (tf::TransformException &ex){
				ROS_ERROR("%s",ex.what());
				continue;
			}
		} else {
			ROS_ERROR("waitForTransform timed out in RoadModel::addSegments");
			break;
		}
	}
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * 					ROAD MODEL
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void RoadModel::getSegmentSearchStart(cv::Point2f &posWorld, float &angle) const {
    float firstTrajPointAt = 0.4f;
    float firstY = dl.poly.atX(firstTrajPointAt);

    // If we are on the left lane, the trajectory could be laneWidth [m] on our right side
    bool firstPtOnStreet = (firstY > (-1.2f * laneWidth)) & (firstY < 0.2f * laneWidth);
    bool angleOk = fabsf(dl.poly.getFstDeviationAtX(firstTrajPointAt)) < M_PI_4;

    if(firstPtOnStreet && angleOk && (noNewSegmentsCtr < 6)) {
        posWorld.x = firstTrajPointAt;
        posWorld.y = firstY;
        angle = dl.poly.getFstDeviationAtX(firstTrajPointAt);

//		ROS_INFO("Start search angle is %.1f[deg]", angle * 180.f / M_PI);

		// TODO: what do to if there is an intersection very close? -> is this handled by BT?
	} else {
		// If we have not found a suitable segment, return some default values
		posWorld.x = 0.3f;
		posWorld.y = 0.f;
		angle = 0.f;

//        ROS_INFO("FirstPointOk? %s AngleOk? %s", (firstPtOnStreet ? "Yes" : "No"), (angleOk ? "Yes" : "No"));
	}
}

bool RoadModel::segmentFitsToPrevious(Segment *previousSegment, Segment *segmentToAdd, bool isFirstSegment) {
	if(segmentToAdd->probablity < 0.2f) {
//		ROS_INFO("Segments probability is too low");
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

	return true;
}

bool RoadModel::addSegments(std::vector<Segment> &newSegments, ros::Time timestamp) {
	if(newSegments.empty()) {
//		ROS_INFO("!!! No new segments");
		noNewSegmentsCtr++;
		segmentsToDl.clear(); // TODO: should we really do this here?
        return false;
	} else {
		noNewSegmentsCtr = 0;

		// 0) compare angle of previous first segment and new first segment
		if(!segmentsToDl.empty()) {
			if(fabsf(newSegments.at(0).angleTotal - segmentsToDl.at(0).angleTotal) > M_PI_4) {
				ROS_INFO("Diff between first segment angles is too big");
				return false;
			}

			// Compare positions of lane markings
			float leftOff = newSegments.at(0).leftPosW.y - segmentsToDl.at(0).leftPosW.y;
			float midOff = newSegments.at(0).midPosW.y - segmentsToDl.at(0).midPosW.y;
			float rightOff = newSegments.at(0).rightPosW.y - segmentsToDl.at(0).rightPosW.y;

			bool leftIsOff = fabsf(leftOff) > (laneWidth * .8f);
			bool midIsOff = fabsf(midOff) > (laneWidth * .8f);
			bool rightIsOff = fabsf(rightOff) > (laneWidth * .8f);

			if(leftIsOff) {
				ROS_INFO("Left line is off by %.2f", leftOff);
			}

			if(midIsOff) {
				ROS_INFO("Mid line is off by %.2f", midOff);
			}

			if(rightIsOff) {
				ROS_INFO("Right line is off by %.2f", rightOff);
			}

			// TODO: if lines are off, check if mid is close to oldLeft or oldRight
			if(leftIsOff && midIsOff && rightIsOff) {
				if(		((leftOff < 0.f) && (midOff < 0.f) && (rightOff < 0.f)) ||
						((leftOff > 0.f) && (midOff > 0.f) && (rightOff > 0.f))) {
					// we can assume that all lines are more than (laneWidth * .8 off)
					// shift all lines to positive y
					float shiftY = (leftOff + midOff + rightOff) / -3.f;

					for(int i = 0; i < newSegments.size(); i++) {
						// TODO: world and image coordinates are not matching anymore!! check if we need these later or its ok
						newSegments.at(i).leftPosW.y += shiftY;
						newSegments.at(i).midPosW.y += shiftY;
						newSegments.at(i).rightPosW.y += shiftY;
						newSegments.at(i).positionWorld.y += shiftY;
					}

				} else {
					ROS_WARN("Weird line offset");
				}
			}

		}


		// 1) Build polynom from new segments
		std::vector<cv::Point2f> ptsWorldForPoly;

		for(auto s : newSegments) {
			ptsWorldForPoly.push_back(s.positionWorld);

			// TODO: can probably be removed because we do not find intersections
			if(s.isIntersection()) {
				ROS_INFO("Got an intersection segment");
				break;
			}
		}

		Polynom newPoly(polyOrder, ptsWorldForPoly);

		// determine detection range
		float newDetectionRangeWorld =
				std::max_element(
						ptsWorldForPoly.begin(),
						ptsWorldForPoly.end(),
						[](cv::Point2f &fst, cv::Point2f &scd) { return fst.x < scd.x; }
				)->x;

		if(newDetectionRangeWorld < 0.8f) {
			ROS_INFO("Detection range too short");
            return false;
		}

		// 2) Compare the current driving line polynom with the old one
		// TODO: the old polynom is not perfectly correct since we ignore the cars movement
		float compareRange = std::min(dl.detectionRange, newDetectionRangeWorld);
		compareRange = 1.f;
		int steps = 10;
		float step = compareRange / steps;
		float error = 0.f;

		for(int i = 1; i < steps; i++) {
			auto newY = newPoly.atX(i * step);
			auto oldY = dl.poly.atX(i * step);
			auto diff = newY - oldY;
			error += std::sqrt(diff*diff);
		}

//		ROS_INFO("---");
//		ROS_INFO("Old range = %.1f", dl.detectionRange);
//		ROS_INFO("New range = %.1f", newDetectionRange);
//		ROS_INFO("Poly diff = %.2f", error);

		// TODO: maybe take history over polys and weight them based on age

		dl = DrivingLane(newPoly, newDetectionRangeWorld, timestamp);
		segmentsToDl = newSegments;
        return true;
	}
}

std::vector<tf::Stamped<tf::Point>> RoadModel::getDrivingLinePts() {
	float currentX = 0.4f;
	std::vector<tf::Stamped<tf::Point>> linePts;

	while(currentX < dl.detectionRange) {
		tf::Stamped<tf::Point> pt;

		pt.setX(currentX);
		pt.setY(dl.poly.atX(currentX));

		linePts.push_back(pt);

		currentX += 0.1f;
	}

	return linePts;
}

} // namespace drive_ros_image_recognition
