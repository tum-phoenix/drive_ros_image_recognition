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
	} else {
		// If we have not found a suitable segment, return some default values
		posWorld.x = 0.3f;
		posWorld.y = 0.f;
		angle = 0.f;
	}
}

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

		ROS_INFO("startPt (%.2f, %.2f)", startPt.x, startPt.y);
		ROS_INFO("  endPt (%.2f, %.2f)", endPt.x, endPt.y);
		ROS_INFO(" angle = %.2f", atan2(endPt.y - startPt.y, endPt.x - startPt.x));

		if(startPt.x < 0.2f) {
			ROS_INFO("  remove first point");
		} else {
			positions.push_back(startPt);
			angles.push_back(atan2(endPt.y - startPt.y, endPt.x - startPt.x));
		}
	}

	ROS_INFO("Returning %lu positions from %lu segments", positions.size(), odomPts.size() / 2);

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
	std::vector<cv::Point2f> rearAxisPts;
	std::vector<tf::Stamped<tf::Point>> odomPts;

	rearAxisPts.push_back(seg.positionWorld);
	rearAxisPts.push_back(seg.endPositionImage);

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

	bool isFirstSegment = index == 0;

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

			if(fabs(newSegments.at(0).positionWorld.y - segmentsToDl.at(0).positionWorld.y) > .6f*laneWidth) {
				ROS_WARN("Start position off");
				return false;
			}

#if 0
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
#endif

		}


		// 1) Build polynom from new segments
		std::vector<cv::Point2f> ptsWorldForPoly;

		for(auto s : newSegments) {
			ptsWorldForPoly.push_back(s.positionWorld);
			ROS_INFO("(%.2f, %.2f)", s.positionWorld.x, s.positionWorld.y);

			cv::Point2f segmentMidPtWorld = s.positionWorld;
			segmentMidPtWorld.x += (s.length * 0.5 * cos(s.angleTotal));
			segmentMidPtWorld.y += (s.length * 0.5 * sin(s.angleTotal));

			ROS_INFO("o(%.2f, %.2f)", segmentMidPtWorld.x, segmentMidPtWorld.y);
			ptsWorldForPoly.push_back(segmentMidPtWorld);

			// TODO: can probably be removed because we do not find intersections
			if(s.isIntersection()) {
				ROS_INFO("Got an intersection segment");
				break;
			}
		}

		ROS_INFO("+++ ptsWorldForPoly.size() = %lu", ptsWorldForPoly.size());

		// if we have only two points, we want a polynom of order 1
		int polyOrder = (ptsWorldForPoly.size() > 2) ? defaultPolyOrder : 1;
		Polynom newPoly(polyOrder, ptsWorldForPoly);

		// determine detection range
		auto maxElem =
				std::max_element(
						ptsWorldForPoly.begin(),
						ptsWorldForPoly.end(),
						[](cv::Point2f &fst, cv::Point2f &scd) { return fst.x < scd.x; }
				);

//		float newDetectionRangeWorld = maxElem->x;

		float newDetectionRangeWorld = newSegments.back().positionWorld.x + newSegments.back().length;

		if(newDetectionRangeWorld < 0.5f) {
			ROS_INFO("Detection range too short");
			driveStraight = true;
            return false;
		}
		driveStraight = false;

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
		ROS_INFO("Poly diff = %.2f", error);

		float fstDev = newPoly.getFstDeviationAtX(newSegments.at(0).positionWorld.x);
		if(fabsf(atan(fstDev) - newSegments.at(0).angleTotal) > M_PI_2) {
			ROS_WARN("Polynom' does not match angle of first segment");
			ROS_INFO("fstDev = %.2f, angle of first segment = %.2f", atan(fstDev), newSegments.at(0).angleTotal);
			// TODO: what to do here?
			return false;
		}

		dl = DrivingLane(newPoly, newDetectionRangeWorld, timestamp);
		segmentsToDl = newSegments;
        return true;
	}
}

Polynom RoadModel::getDrivingLinePts() {
	std::vector<tf::Stamped<tf::Point>> odomPts, rearAxisPts;

	for(auto s : segmentsToDl) {
		if(s.odomPointsSet) {
			ROS_INFO("  Segment.ttl: %i", s.ttl);
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

	transformOdomPointsToRearAxis(pTfListener, odomPts, rearAxisPts, ros::Time(0)); // TOD: time not used anymore

	std::vector<cv::Point2f> drivingLinePts;
	drivingLinePts.push_back(cv::Point2f(.3f, 0.f));
	for(auto sp : rearAxisPts) {
		drivingLinePts.push_back(cv::Point2f(sp.x(), sp.y()));
	}

	ROS_INFO("Built polynom from %lu points from %lu segments", drivingLinePts.size(), segmentsToDl.size());

	if(drivingLinePts.size() == 1) { // we always add a point in front of the vehicle
		// return a straight line
		drivingLinePts.push_back(cv::Point(.6, 0.f));
	}
	int polyOrder = (drivingLinePts.size() > 2) ? defaultPolyOrder : 1;
	return Polynom(polyOrder, drivingLinePts);
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
