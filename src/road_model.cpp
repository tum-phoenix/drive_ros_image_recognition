#include <ros/ros.h>
#include "drive_ros_image_recognition/road_model.h"

namespace drive_ros_image_recognition {

void RoadModel::getFirstPosW(cv::Point2f &posW, float &angle) const {
	for(auto s : segments) {
		// Find the first segment which is in front of the car
		if(s.positionWorld.x > 0.1f) {
			// If the segment is too far away translate if towards the car
			if(s.positionWorld.x > 0.5f) {
//				ROS_WARN_STREAM("First segment is too far away: " << s.positionWorld);
				float moveDistance = sqrt(s.positionWorld.x*s.positionWorld.x + s.positionWorld.y*s.positionWorld.y);
				moveDistance -= 0.3f;

				posW.x = s.positionWorld.x - (moveDistance * cos(s.angleTotal));
				posW.y = s.positionWorld.y - (moveDistance * sin(s.angleTotal));
			} else {
				posW = s.positionWorld;
			}

			angle = s.angleTotal;
			return;
		}
	}

	// If we have not found a suitable segment, return some default values
	posW.x = 0.3f;
	posW.y = 0.f;
	angle = 0.f;

	return;
}

void RoadModel::addSegments(std::vector<Segment> &newSegments) {
//	ROS_INFO_STREAM(segments.size() << " old segments and " << newSegments.size() << " new ones");
	for(int i = 0; (i < newSegments.size()) && (i < segments.size()); i++) {
		auto deltaAngleDiff = std::abs(newSegments.at(i).angleDiff - segments.at(i).angleDiff);
		auto deltaPosition = newSegments.at(i).positionWorld - segments.at(i).positionWorld;
//		ROS_INFO_STREAM("(" << i << ") deltaAngleDiff = " << (deltaAngleDiff * 180.f / M_PI) << "    deltaPosition = " << deltaPosition);
	}

	segments.clear();
	segments.insert(segments.begin(), newSegments.begin(), newSegments.end());
//	ROS_INFO_STREAM("Num segments: " << segments.size());
}

} // namespace drive_ros_image_recognition
