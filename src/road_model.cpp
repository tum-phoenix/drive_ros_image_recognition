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
	if(noNewSegmentsCtr < 6) {
		posWorld.x = 0.4f;
		posWorld.y = dl.poly.atX(posWorld.x);
		angle = dl.poly.getFstDeviationAtX(posWorld.x);

		ROS_INFO("Start search angle is %.1f[deg]", angle * 180.f / M_PI);

		// TODO: what do to if there is an intersection very close? -> is this handled by BT?
	} else {
		// If we have not found a suitable segment, return some default values
		posWorld.x = 0.3f;
		posWorld.y = 0.f;
		angle = 0.f;
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
			ROS_WARN("First segments angle is too big: %.1f[deg]", segmentToAdd->angleTotal * 180.f / M_PI);
			return false;
		}
	}

	return true;
}

void RoadModel::addSegments(std::vector<Segment> &newSegments, ros::Time timestamp) {
	if(newSegments.empty()) {
		ROS_INFO("!!! No new segments");
		noNewSegmentsCtr++;
	} else {
		noNewSegmentsCtr = 0;

		// 1) Build polynom from new segments
		std::vector<cv::Point2f> ptsForPoly;

		for(auto s : newSegments) {
			ptsForPoly.push_back(s.positionWorld);

			if(s.isIntersection()) {
				ROS_INFO("Got an intersection segment");
				break;
			}
		}

		Polynom newPoly(3, ptsForPoly);

		// TODO: could probably be done in for loop over segments above
		float newDetectionRange =
				std::max_element(
						ptsForPoly.begin(),
						ptsForPoly.end(),
						[](cv::Point2f &fst, cv::Point2f &scd) { return fst.x > fst.y; }
				)->x;

		if(newDetectionRange < 0.5f) {
			return;
		}

		// 2) Compare the current driving line polynom with the one
		// TODO: the old polynom is not perfectly correct since we ignore the cars movement
		float compareRange = std::min(dl.detectionRange, newDetectionRange);
		int steps = 10;
		float step = compareRange / steps;
		float error = 0.f;

		for(int i = 1; i < steps; i++) {
			auto newY = newPoly.atX(i * step);
			auto oldY = dl.poly.atX(i * step);
			auto diff = newY - oldY;
			error += std::sqrt(diff*diff);
		}

		ROS_INFO("---");
		ROS_INFO("Old range = %.1f", dl.detectionRange);
		ROS_INFO("New range = %.1f", newDetectionRange);
		ROS_INFO("Poly diff = %.2f", error);

		// TODO: maybe combine both polys

		dl = DrivingLane(newPoly, newDetectionRange, timestamp);
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
