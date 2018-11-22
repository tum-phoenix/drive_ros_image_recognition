#include <ros/ros.h>
#include "drive_ros_image_recognition/road_model.h"

namespace drive_ros_image_recognition {

void RoadModel::getSegmentSearchStart(cv::Point2f &posWorld, float &angle) const {
	// DEBUG
	for(int i = 0; i < drivingLine.size(); i++) {
		if(drivingLine.at(i).isIntersection()) {
			ROS_INFO_STREAM("=== SearchStart: IntersectionSegment is at " << i << "/" << drivingLine.size());
		}
	}

	auto intersectionSegment = std::find_if(drivingLine.begin(), drivingLine.end(), [](Segment s){ return s.isIntersection(); });
	// If an intersection exists, start searching for segments after it
	if(intersectionSegment != drivingLine.end()) {
		if(intersectionSegment == (drivingLine.end() - 1)) {
			ROS_INFO("Create the intersection exit point");
			// We do not have any segment after the intersection
			tf::Stamped<tf::Point> p;

			try {
				pTfListener->transformPoint("/rear_axis_middle_ground", intersectionSegment->odomPosition, p);
			}
			catch (tf::TransformException &ex){
				ROS_ERROR("%s",ex.what());
			}

			// Guess the intersection exit position
			posWorld.x = p.x() + (1.f * cos(intersectionSegment->angleTotal));
			posWorld.y = p.y() + (1.f * sin(intersectionSegment->angleTotal));
			angle = intersectionSegment->angleTotal;
			ROS_INFO("(%f, %f)", posWorld.x, posWorld.y);
			return;

		} else {
			ROS_INFO("Return segment after intersection");
			// Select the first segment after the intersection
			auto searchStartSeg = intersectionSegment + 1;

			tf::Stamped<tf::Point> p;

			try {
				pTfListener->transformPoint("/rear_axis_middle_ground", searchStartSeg->odomPosition, p);
			}
			catch (tf::TransformException &ex){
				ROS_ERROR("%s",ex.what());
			}

			posWorld.x = p.x();
			posWorld.y = p.y();
			angle = searchStartSeg->angleTotal;
			ROS_INFO("(%f, %f)", posWorld.x, posWorld.y);
			return;
		}
	}

	// If there is no intersection select the first segment
	for(auto s : drivingLine) {
		tf::Stamped<tf::Point> p;

		try {
			pTfListener->transformPoint("/rear_axis_middle_ground", s.odomPosition, p);
		}
		catch (tf::TransformException &ex){
			ROS_ERROR("%s",ex.what());
			continue;
		}

		// Find the first segment which is in front of the car
		if(p.x() > 0.2f && (std::abs(p.y()) < 0.3f)) {
			// If the segment is too far away translate if towards the car
			if(p.x() > 0.5f) {
				//				ROS_WARN_STREAM("First segment is too far away: " << s.positionWorld);
				float moveDistance = sqrt(p.x()*p.x() + p.y()*p.y());
				moveDistance -= 0.3f;

				posWorld.x = p.x() - (moveDistance * cos(s.angleTotal));
				posWorld.y = p.y() - (moveDistance * sin(s.angleTotal));

			} else {
				posWorld.x = p.x();
				posWorld.y = p.y();
			}

			angle = s.angleTotal;
			return;
		}
	}

	// If we have not found a suitable segment, return some default values
	posWorld.x = 0.3f;
	posWorld.y = 0.f;
	angle = 0.f;

	return;
}

std::vector<tf::Stamped<tf::Point>> RoadModel::getDrivingLinePts() {
	// DEBUG
	for(int i = 0; i < drivingLine.size(); i++) {
		if(drivingLine.at(i).isIntersection()) {
			ROS_INFO_STREAM("=== DrivingLinePts: IntersectionSegment is at " << i << "/" << drivingLine.size());
		}
	}

	std::vector<tf::Stamped<tf::Point>> drivingLinePts;
	float drivingLineLength = 0.f;

	// If we found an intersection aim for the exit
	// TODO: actually, we should aim for the entry (and check if we have to stop) and then aim for the exit
//	auto intersectionSegment = std::find_if(drivingLine.begin(), drivingLine.end(), [](Segment s){ return s.isIntersection(); });
//	if(intersectionSegment != drivingLine.end()) {
//		ROS_INFO("Intersection (%s) exists - aim for exit",
//				intersectionSegment->segmentType == SegmentType::INTERSECTION_GO_STRAIGHT ? "GO" : "STOP");
//
//		if(intersectionSegment == (drivingLine.end() - 1)) {
//			// We do not have any segment after the intersection
//			tf::Stamped<tf::Point> p;
//
//			try {
//				pTfListener->transformPoint("/rear_axis_middle_ground", intersectionSegment->odomPosition, p);
//			}
//			catch (tf::TransformException &ex){
//				ROS_ERROR("%s",ex.what());
//			}
//
//			// Guess the intersection exit position
//			tf::Stamped<tf::Point> interpolatedPt;
//			interpolatedPt.setX(p.x() + (1.f * cos(intersectionSegment->angleTotal)));
//			interpolatedPt.setY(p.y() + (1.f * sin(intersectionSegment->angleTotal)));
//			drivingLinePts.push_back(interpolatedPt);
//			auto dist = sqrt(interpolatedPt.x()*interpolatedPt.x() + interpolatedPt.y()*interpolatedPt.y());
//			drivingLineLength = dist;
//
//		} else {
//			// Select the first segment after the intersection
//			auto searchStartSeg = intersectionSegment + 1;
//
//			tf::Stamped<tf::Point> p;
//
//			try {
//				pTfListener->transformPoint("/rear_axis_middle_ground", searchStartSeg->odomPosition, p);
//			}
//			catch (tf::TransformException &ex){
//				ROS_ERROR("%s",ex.what());
//			}
//
//			auto dist = sqrt(p.x()*p.x() + p.y()*p.y());
//			drivingLineLength = dist;
//			drivingLinePts.push_back(p);
//		}
//	}

	// The actual driving line (if an intersection exists, we ignore segments before it)
	for(auto s : drivingLine) {
		tf::Stamped<tf::Point> p;

		try {
			pTfListener->transformPoint("/rear_axis_middle_ground", s.odomPosition, p);
		}
		catch (tf::TransformException &ex){
			ROS_ERROR("%s",ex.what());
			continue;
		}

		auto dist = sqrt(p.x()*p.x() + p.y()*p.y());
		if(dist > drivingLineLength) {
			drivingLinePts.push_back(p);
			drivingLineLength = dist;
		}
	}

	return drivingLinePts;
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
	// Transform drivingLine to /rear_axis_middle_ground frame
	// TODO

	// Compare the current driving line with the new segment
	// TODO

	// Add the new segments to the driving line
	std::vector<Segment> updatedDrivingLine;
	float drivingLineLength = 0.f;

	for(auto s : newSegments) {
		// The new segments already had a plausibility check

		// Transform the points to the /odom frame
		tf::Stamped<tf::Point> p;
		p.frame_id_ = "/rear_axis_middle_ground";
		p.stamp_ = timestamp;
		p.setX(s.positionWorld.x);
		p.setY(s.positionWorld.y);
		p.setZ(0.0);

//			target_frame	The frame into which to transform
//			source_frame	The frame from which to transform
//			time	The time at which to transform
//			timeout	How long to block before failing
		if(pTfListener->waitForTransform("/odom", p.frame_id_, timestamp, ros::Duration(0.1))) {
			try {
				pTfListener->transformPoint("/odom", p, s.odomPosition);

			}
			catch (tf::TransformException &ex){
				ROS_ERROR("%s",ex.what());
				continue;
			}

			updatedDrivingLine.push_back(s);
			drivingLineLength = sqrt(p.x()*p.x() + p.y()*p.y());
		} else {
			ROS_ERROR("waitForTransform timed out in RoadModel::addSegments");
			break;
		}
	}

	// If the new driving line is too short add some segments from the previous one
	if(drivingLineLength < 1.0f) {
		for(auto s : drivingLine) {
			tf::Stamped<tf::Point> p;

			// Transform the point from /odom to /rear_axis_middle_ground
			try {
				pTfListener->transformPoint("/rear_axis_middle_ground", s.odomPosition, p);
			}
			catch (tf::TransformException &ex){
				ROS_ERROR("%s",ex.what());
				continue;
			}

			auto dist = sqrt(p.x()*p.x() + p.y()*p.y());
			if(dist > drivingLineLength) {
				// Only use the previous driving line up to a certain distance
				if(dist < 1.8f) {
					// Check if the old segments fits to the current driving line
					if(segmentFitsToPrevious(
							updatedDrivingLine.empty() ? nullptr : &updatedDrivingLine.back(),
							&s, updatedDrivingLine.empty())) {
						updatedDrivingLine.push_back(s);
					}

				} else {
					// Break if the old segment is too far away
					break;
				}
			}
		}
	}

	drivingLine = updatedDrivingLine;
}

} // namespace drive_ros_image_recognition
