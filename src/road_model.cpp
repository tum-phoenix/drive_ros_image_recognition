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

	transformOdomPointsToRearAxis(pTfListener, odomPts, rearAxisPts);

	positions.clear();
	angles.clear();

	for(int i = 0; i < rearAxisPts.size(); i += 2) {
		cv::Point2f startPt(rearAxisPts.at(i).x(),   rearAxisPts.at(i).y());
		cv::Point2f endPt  (rearAxisPts.at(i+1).x(), rearAxisPts.at(i+1).y());

		if(startPt.x < 0.f) {
			ROS_INFO("  remove first point");
		} else {
			positions.push_back(startPt);
			angles.push_back(atan2(endPt.y - startPt.y, endPt.x - startPt.x));
		}
	}

	ROS_INFO("getSegmentPositions: Returning %lu positions from %lu segments", positions.size(), odomPts.size() / 2);

	if(!positions.empty()) {
		ROS_INFO("  first position at (%.2f, %.2f)", positions.begin()->x, positions.begin()->y);
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
	ROS_INFO("  updateSegmentAtIndex %u", index);

	if(index < segmentsToDl.size()) {
		segmentsToDl.at(index) = seg;
	} else if(index == segmentsToDl.size()) {
		segmentsToDl.push_back(seg);
	} else {
		ROS_WARN("  index too high");
	}
}

bool RoadModel::segmentFitsToPrevious(Segment *segmentToAdd, int index) {
    bool isFirstSegment = (index == 0);

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
	if(segmentsToDl.empty()) {
		return true;
	}
    float error = .0f;
    error += std::fabs(segmentToAdd->positionWorld.y - currentDrivingLinePoly.atX(segmentToAdd->positionWorld.x));
    error += std::fabs(segmentToAdd->endPositionWorld.y - currentDrivingLinePoly.atX(segmentToAdd->endPositionWorld.x));

    if(error > maxPolyError) {
    	ROS_INFO("  error based on polynom: %.3f", error);
        return false;
    }

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

  ROS_INFO("  Built %s line from %lu points with limits [%.2f, %.2f]",
		  (leftLine ? "left" : "right"), linePts.size(),
		  std::min_element(linePts.begin(), linePts.end(), [](cv::Point2f &a, cv::Point2f &b) { return a.x < b.x; })->x,
		  std::max_element(linePts.begin(), linePts.end(), [](cv::Point2f &a, cv::Point2f &b) { return a.x < b.x; })->x);

  return true;
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

  ROS_INFO("Built driving line polynom from %lu points from %lu segments", drivingLinePts.size(), segmentsToDl.size());

  if(drivingLinePts.size() < 2) {
    // return a straight line
    drivingLinePts.push_back(cv::Point(.6, 0.f));
  }

  // 3) Build the polynom
  int polyOrder = (drivingLinePts.size() > 2) ? defaultPolyOrder : 1;
  drivingLine = Polynom(polyOrder, drivingLinePts);

  return drivingLinePts.back().x;
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
