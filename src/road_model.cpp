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
    std::vector<tf::Stamped<tf::Point>> &rearAxisPts)
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
		std::vector<tf::Stamped<tf::Point>> &odomPts)
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
				if(odomPts.empty()) {
					ROS_WARN("not using first segment");
				}
				break;
			}
		} else {
			ROS_WARN("odom pts not set");
		}
	}

	if(!transformOdomPointsToRearAxis(pTfListener, odomPts, rearAxisPts)) {
		return;
	}

	positions.clear();
	angles.clear();

	for(int i = 0; i < rearAxisPts.size(); i += 2) {
		cv::Point2f startPt(rearAxisPts.at(i).x(),   rearAxisPts.at(i).y());
		cv::Point2f endPt  (rearAxisPts.at(i+1).x(), rearAxisPts.at(i+1).y());

		if(startPt.x < 0.f) {
//			ROS_INFO("  remove first point");
		} else {
			positions.push_back(startPt);
			angles.push_back(atan2(endPt.y - startPt.y, endPt.x - startPt.x));
		}
	}

	if(positions.empty()) {
		return;
	}

//	ROS_INFO(" *** First segment is at position (%.2f,%.2f) with angle %.f",
//			positions.begin()->x, positions.begin()->y, angles.at(0) / M_PI * 180.f);
	if(positions.begin()->x > .8f) {
//		ROS_INFO(" *** Create an artificial segment at position 0");
		// Sometimes the odometry fails and our segments move to the front, so we create a new one at the first position
		positions.insert(positions.begin(), cv::Point2f(positions.begin()->x * .5f, positions.begin()->y * .5f));
		angles.insert(angles.begin(), angles.at(0) * .5f);
		segmentsToDl.insert(segmentsToDl.begin(), Segment());
//		ROS_INFO(" *** Now is (%.2f,%.2f) with angle %.f",
//				positions.begin()->x, positions.begin()->y, angles.at(0) / M_PI * 180.f);
	}

//	ROS_INFO("getSegmentPositions: Returning %lu positions from %lu segments", positions.size(), odomPts.size() / 2);

	if(!positions.empty()) {
//		ROS_INFO("  first position at (%.2f, %.2f)", positions.begin()->x, positions.begin()->y);
	}

}

void RoadModel::setOdomPointsForSegments() {
	std::vector<cv::Point2f> rearAxisPts;
	std::vector<tf::Stamped<tf::Point>> odomPts;

	for(int i = 0; i < segmentsToDl.size(); i++) {
		if(segmentsToDl.at(i).odomPointsSet) {
			continue;
		}

		rearAxisPts.clear();
		odomPts.clear();

		rearAxisPts.push_back(segmentsToDl.at(i).positionWorld); // idx 0
		rearAxisPts.push_back(segmentsToDl.at(i).endPositionWorld); // idx 1
		if(segmentsToDl.at(i).leftLineSet) {
			rearAxisPts.push_back(segmentsToDl.at(i).leftLineStart); // idx 2
			rearAxisPts.push_back(segmentsToDl.at(i).leftLineEnd); // idx 3
		}
		if(segmentsToDl.at(i).rightLineSet) {
			rearAxisPts.push_back(segmentsToDl.at(i).rightLineStart); // idx 4
			rearAxisPts.push_back(segmentsToDl.at(i).rightLineEnd); // idx 5
		}

		if(transformRearAxisPointsToOdom(pTfListener, rearAxisPts, odomPts)) {
			size_t odomIdx = 0;
			segmentsToDl.at(i).odomStart = odomPts.at(odomIdx++);
			segmentsToDl.at(i).odomEnd = odomPts.at(odomIdx++);
			if(segmentsToDl.at(i).leftLineSet) {
				segmentsToDl.at(i).odomLeftLineStart = odomPts.at(odomIdx++);
				segmentsToDl.at(i).odomLeftLineEnd = odomPts.at(odomIdx++);
			}
			if(segmentsToDl.at(i).rightLineSet) {
				segmentsToDl.at(i).odomRightLineStart = odomPts.at(odomIdx++);
				segmentsToDl.at(i).odomRightLineEnd = odomPts.at(odomIdx++);
			}
			segmentsToDl.at(i).creationTimestamp = odomPts.at(0).stamp_;
			segmentsToDl.at(i).odomPointsSet = true;
		}
	}
}

void RoadModel::updateSegmentAtIndex(Segment &seg, int index) {
	if(index < segmentsToDl.size()) {
		segmentsToDl.at(index) = seg;
	} else if(index == segmentsToDl.size()) {
		segmentsToDl.push_back(seg);
	} else {
		ROS_WARN("  index too high");
	}
}

bool RoadModel::segmentFitsToPrevious(Segment *segmentToAdd, int index, bool &possibleIntersection) {
    bool isFirstSegment = (index == 0);
    possibleIntersection = false;

	if((!isFirstSegment) && (index > segmentsToDl.size())) {
		ROS_INFO("  index > segmentsToDl.size()");
		return false;
	}

	Segment *previousSegment = (isFirstSegment ? nullptr : &(segmentsToDl.at(index - 1)));

	if(segmentToAdd->probablity < 0.2f) {
		ROS_INFO("  segments probability is too low");
		return false;
	}

	if(segmentToAdd->length < 0.05f) {
		ROS_INFO("  segment length < 0.05");
		return false;
	}

	if(!isFirstSegment) {
		float angleVar = M_PI / 7.0f; // TODO move to config
		if(std::abs(previousSegment->angleTotal - segmentToAdd->angleTotal) > angleVar) {
			ROS_INFO_STREAM("  too much angle difference: " << (segmentToAdd->angleDiff * 180.f / M_PI) << "[deg]");
			if(std::abs(previousSegment->angleTotal - segmentToAdd->angleTotal) > (80.f / 180.f * M_PI)) {
				ROS_INFO("  Maybe found an INTERSECTION?");
				possibleIntersection = true;
			}
			return false;
		}
	} else {
		// The first segment should points into the cars driving direction
		if(std::abs(segmentToAdd->angleTotal) > M_PI / 4.0f) {
			ROS_WARN("  first segments angle is too big: %.1f[deg]", segmentToAdd->angleTotal * 180.f / M_PI);
			return false;
		}
	}

    // Compare to previous polynom
    // TODO: quick hack. think this through
//	if(segmentsToDl.empty()) {
//		return true;
//	}
//    float error = .0f;
//    error += std::fabs(segmentToAdd->positionWorld.y - currentDrivingLinePoly.atX(segmentToAdd->positionWorld.x));
//    error += std::fabs(segmentToAdd->endPositionWorld.y - currentDrivingLinePoly.atX(segmentToAdd->endPositionWorld.x));
//
//    if(error > maxPolyError) {
//    	ROS_INFO("  error based on polynom: %.3f", error);
//        return false;
//    }

	return true;
}

bool RoadModel::getLaneMarkings(Polynom &line, bool leftLine) {
  std::vector<tf::Stamped<tf::Point>> odomPts, rearAxisPts;

  // 1) Convert the odom pointst to rear_axis_middle_ground
  int segCtr = 0; // DEBUG
  for(auto s : segmentsToDl) {
	  segCtr++;
    if(s.odomPointsSet) {
//      ROS_INFO("  laneMarking for Seg #%u on %s side with TTL=%i", segCtr, (leftLine ? "left" : "right"), s.ttl);
//      if(leftLine)
//    	  ROS_INFO("    Left line set? %s", (s.leftLineSet ? "yes" : "no"));
//      else
//    	  ROS_INFO("    Right line set? %s", (s.rightLineSet ? "yes" : "no"));
      if(s.ttl > 0) {
        if(leftLine && s.leftLineSet) {
          odomPts.push_back(s.odomLeftLineStart);

          auto odomMid = s.odomLeftLineStart;
          odomMid.setX(odomMid.x() + .5f * (s.odomLeftLineEnd.x() - s.odomLeftLineStart.x()));
          odomMid.setY(odomMid.y() + .5f * (s.odomLeftLineEnd.y() - s.odomLeftLineStart.y()));
          odomPts.push_back(odomMid);

          odomPts.push_back(s.odomLeftLineEnd);
        }
        if(!leftLine && s.rightLineSet) {
          odomPts.push_back(s.odomRightLineStart);

          auto odomMid = s.odomRightLineStart;
          odomMid.setX(odomMid.x() + .5f * (s.odomRightLineEnd.x() - s.odomRightLineStart.x()));
          odomMid.setY(odomMid.y() + .5f * (s.odomRightLineEnd.y() - s.odomRightLineStart.y()));
          odomPts.push_back(odomMid);

          odomPts.push_back(s.odomRightLineEnd);
        }
      } else {
        break;
      }
    } else {
      ROS_WARN("odom pts not set");
    }
  }

  if(odomPts.empty()) {
    return false;
  }
  transformOdomPointsToRearAxis(pTfListener, odomPts, rearAxisPts);

  // 2) Collect points for the polynoms
  std::vector<cv::Point2f> linePts;

  for(auto p : rearAxisPts) {
    linePts.push_back(cv::Point2f(p.x(), p.y()));
  }

  if(linePts.size() < 4) {
    return false;
  }

  // 3) Build the polynom
  line = Polynom(defaultPolyOrder, linePts);

  // Sometimes all coeffs are zero. TODO: Why is this happening
  bool polyValid = false;
  for(auto c : line.getCoeffs()) {
	  if(c != 0.f) {
		  polyValid = true;
	  }
  }

//  ROS_INFO("  Built %s line from %lu points with limits [%.2f, %.2f]",
//		  (leftLine ? "left" : "right"), linePts.size(),
//		  std::min_element(linePts.begin(), linePts.end(), [](cv::Point2f &a, cv::Point2f &b) { return a.x < b.x; })->x,
//		  std::max_element(linePts.begin(), linePts.end(), [](cv::Point2f &a, cv::Point2f &b) { return a.x < b.x; })->x);

  return polyValid;
}

void RoadModel::setDefaultPolyOrder(int o) {
	defaultPolyOrder = o;
	polyHistoryFilled = false;
	polyHistoryIdx = 0;
	polyHistory.clear();
	polyRangeHistory.clear();
}

float RoadModel::getDrivingLine(Polynom &drivingLine) {
	std::vector<tf::Stamped<tf::Point>> odomMidPts, rearAxisMidPts;

	// 1) Convert the odom pointst to rear_axis_middle_ground
	for(auto s : segmentsToDl) {
		if(s.odomPointsSet) {
			if(s.ttl > 0) {
				odomMidPts.push_back(s.odomStart);
				// Add a point in the middle of the segment
				auto odomMid = s.odomStart;
				odomMid.setX(odomMid.x() + .5f * (s.odomEnd.x() - s.odomStart.x()));
				odomMid.setY(odomMid.y() + .5f * (s.odomEnd.y() - s.odomStart.y()));
				odomMidPts.push_back(odomMid);
			} else {
				break;
			}
		} else {
			ROS_WARN("odom pts not set");
		}
	}

	transformOdomPointsToRearAxis(pTfListener, odomMidPts, rearAxisMidPts);

	// 2) Collect points for the polynoms
	std::vector<cv::Point2f> drivingLinePts;
	// we always add a point in front of the vehicle
	drivingLinePts.push_back(cv::Point2f(.3f, 0.f));
	for(auto sp : rearAxisMidPts) {
		drivingLinePts.push_back(cv::Point2f(sp.x(), sp.y()));
	}

	//  ROS_INFO("Built driving line polynom from %lu points from %lu segments", drivingLinePts.size(), segmentsToDl.size());

	if(drivingLinePts.size() < 2) {
		// return a straight line
		drivingLinePts.push_back(cv::Point(.6, 0.f));
	}

	// 3) Build the polynom
	int polyOrder = (drivingLinePts.size() > 2) ? defaultPolyOrder : 1;
	polyOrder = defaultPolyOrder;
	Polynom newDrivingLine = Polynom(polyOrder, drivingLinePts);
	float newDetectionRange = drivingLinePts.back().x;

	if(!polyHistoryFilled) {
		polyHistory.push_back(newDrivingLine);
		polyRangeHistory.push_back(newDetectionRange);

		// For now we return the current poly
		currentDrivingLinePoly = newDrivingLine;
		currentDetectionRange = newDetectionRange;

		if(polyHistory.size() == polyHistoryLen) {
			polyHistoryFilled = true;
		}
	} else {
		if(newDrivingLine.getOrder() == defaultPolyOrder) {
			// Add new poly to history
			polyHistory.at(polyHistoryIdx) = newDrivingLine;
			polyRangeHistory.at(polyHistoryIdx) = newDetectionRange;
			polyHistoryIdx++;
			polyHistoryIdx = polyHistoryIdx % polyHistoryLen;
		}

		// Build average over poly history using the coefficients
		std::vector<float> avgCoeffs(defaultPolyOrder+1, .0f);
		for(auto p : polyHistory) {
			for(int i = 0; i < p.getCoeffs().size(); i++) {
				avgCoeffs.at(i) = avgCoeffs.at(i) + p.getCoeffs().at(i);
			}
		}
		for(int i = 0; i < avgCoeffs.size(); i++) {
			avgCoeffs.at(i) = avgCoeffs.at(i) / polyHistoryLen;
		}

		// Return the poly avg
		currentDrivingLinePoly = Polynom(avgCoeffs);
		currentDetectionRange = std::accumulate(polyRangeHistory.begin(), polyRangeHistory.end(), .0f) / polyHistoryLen;
	}

#if 0
	if(newDetectionRange > .05f) {
		// 4) Compare to previous polynom
		float compareRange = std::min(currentDetectionRange, newDetectionRange);
		int numComparisons = 10;
		float step = compareRange / numComparisons;
		float error = .0f;
		for(int i = 0; i < numComparisons; i++) {
			float xNow = step + (step * i);
			error += fabsf(newDrivingLine.atX(xNow) - currentDrivingLinePoly.atX(xNow));
		}
		error /= numComparisons;
		ROS_INFO("=== Polynom error = %.3f in range %.2f", error, compareRange);
		if(error > maxPolyError) {
			ROS_WARN("Polynom error too large");
			currentPolyAge++;
		} else {
			currentDrivingLinePoly = newDrivingLine;
			currentDetectionRange = newDetectionRange;
			currentPolyAge = 0;
		}
	} else {
		currentPolyAge++;
	}
#endif

	drivingLine = currentDrivingLinePoly;
	return currentDetectionRange;
}

void RoadModel::decreaseAllSegmentTtl() {
	for(auto it = segmentsToDl.begin(); it != segmentsToDl.end(); ++it) {
		it->ttl = it->ttl - 1;
	}

	for(auto it = intersections.begin(); it != intersections.end(); ++it) {
		it->confidence = it->confidence - .2f;
	}

	auto lastElemToKeep = std::remove_if(intersections.begin(), intersections.end(), [](Intersection &i) { return i.confidence <= 0.f; });
	intersections.erase(lastElemToKeep, intersections.end());
}

void RoadModel::decreaseSegmentTtl(int index) {
	if(index < segmentsToDl.size()) {
		segmentsToDl.at(index).ttl = segmentsToDl.at(index).ttl - 1;
	}
}

void RoadModel::addIntersectionAt(float x, float confidence, bool hasStopLine) {
	intersections.push_back(Intersection(x, confidence, hasStopLine));
}

bool RoadModel::getIntersections(std::vector<tf::Stamped<tf::Point>> &positions, std::vector<float> &confidences,
		std::vector<bool> &hasStopLine, Polynom &drivingLine) {
	if(intersections.empty()) {
		return false;
	}

	// 1) Convert distances to positions
	for(auto it = intersections.begin(); it != intersections.end(); ++it) {
		if(!it->odomSet) {
			tf::Stamped<tf::Point> rearAxisPt, odomPt;
			rearAxisPt.frame_id_ = "rear_axis_middle_ground";
			rearAxisPt.stamp_ = ros::Time(0);
			rearAxisPt.setX(it->distanceTo);
			rearAxisPt.setY(drivingLine.atX(it->distanceTo));
			rearAxisPt.setZ(0.0f);

			if(transformRearAxisToOdom(pTfListener, rearAxisPt, odomPt)) {
				it->odomPosition = odomPt;
				it->odomSet = true;
			}
		}
	}

	// 2) Combine intersection positions which are close together
	for(auto i = intersections.begin(); i != intersections.end(); ++i) {
		for(auto j = i + 1; j != intersections.end(); ++j) {
			if(i->odomSet && j->odomSet && (i->confidence > 0.f) && (j->confidence > 0.f)) {
				// calculate distance between points
				auto xDist = i->odomPosition.x() - j->odomPosition.x();
				auto yDist = i->odomPosition.y() - j->odomPosition.y();
				auto dist = sqrtf(xDist*xDist + yDist*yDist);
				if(dist < .3f) {
					ROS_INFO("  fuse two intersection positions");
					i->confidence = i->confidence + .3f;
					j->confidence = -1.f;
				}
			}
		}
	}

	// Remove the intersection we just merged
	auto lastElemToKeep = std::remove_if(intersections.begin(), intersections.end(), [](Intersection &i) { return i.confidence <= 0.f; });
	intersections.erase(lastElemToKeep, intersections.end());

	// 3) Transform all odomPosition to rearAxis and return them
	std::vector<tf::Stamped<tf::Point>> odomPts, rearAxisPts;
	for(auto i : intersections) {
		if(i.odomSet) {
			odomPts.push_back(i.odomPosition);
		}
	}

	if(transformOdomPointsToRearAxis(pTfListener, odomPts, rearAxisPts)) {
		for(int i = 0; i < rearAxisPts.size(); i++) {
			if(rearAxisPts.at(i).x() < .3f) {
				intersections.at(i).confidence = -1.f;
			} else {
				positions.push_back(rearAxisPts.at(i));
				confidences.push_back(intersections.at(i).confidence);
				hasStopLine.push_back(intersections.at(i).stopLineFound);
			}
		}
	} else {
		ROS_ERROR("Error while transforming the intersection positions to rear axis");
		return false;
	}

	return !positions.empty();
}

} // namespace drive_ros_image_recognition
