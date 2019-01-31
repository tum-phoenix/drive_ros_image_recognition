#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <chrono>
#include <pluginlib/class_list_macros.h>
#include <eigen3/Eigen/Dense>
#include <random>
#include <functional>
#include <sstream>
#include <tf/transform_datatypes.h>
#include "drive_ros_image_recognition/line_detection.h"
#include "drive_ros_image_recognition/geometry_common.h"
#include "drive_ros_msgs/RoadLine.h"
#include "drive_ros_msgs/simple_trajectory.h"
#include "drive_ros_msgs/DrivingLine.h"
#include "drive_ros_msgs/DetectedIntersection.h"

namespace drive_ros_image_recognition {

LineDetection::LineDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh)
    : nh_(nh)
    , pnh_(pnh)
    , imageTransport_(nh)
    , laneWidthWorld_(0.4)
	, laneVar_(0.1)
    , segmentLength_(0.2)
	, maxSenseRange_(1.6)
	, maxRansacInterations_(200)
    , image_operator_()
    , dsrv_server_()
    , dsrv_cb_(boost::bind(&LineDetection::reconfigureCB, this, _1, _2))
    , roadModel(&tfListener_, laneWidthWorld_)
{
}

LineDetection::~LineDetection() {
}

///
/// \brief LineDetection::init
/// Sets dynamic reconfigure server, subscribes to camera image, inits image_operator.
/// \return true for success, false for failure.
///
bool LineDetection::init() {
    // dynamic reconfigure
    dsrv_server_.setCallback(dsrv_cb_);

    //subscribe to homographied image
    homographiedImageSubscriber_ = imageTransport_.subscribe("homographied_img_in", 3,
                                                             &LineDetection::homographiedImageCallback, this);
    ROS_INFO_STREAM("Subscribed homographied image transport to topic " << homographiedImageSubscriber_.getTopic());

    odometrySub = pnh_.subscribe("odom_topic", 3, &LineDetection::odometryCallback, this);
    ROS_INFO("Subscribing to odometry on topic '%s'", odometrySub.getTopic().c_str());

    drivingLinePub = nh_.advertise<drive_ros_msgs::DrivingLine>("driving_line_topic", 1);
    ROS_INFO("Publish driving line on topic '%s'", drivingLinePub.getTopic().c_str());

    detectedIntersectionsPub = nh_.advertise<drive_ros_msgs::DetectedIntersection>("detected_intersection_topic", 1);
    ROS_INFO("Publish detected intersection on topic '%s'", detectedIntersectionsPub.getTopic().c_str());

#ifdef PUBLISH_DEBUG
    debugImgPub_ = imageTransport_.advertise("debug_image", 3);
    ROS_INFO_STREAM("Publishing debug image on topic " << debugImgPub_.getTopic());
#endif

    // common image operations
    if(!image_operator_.init()) {
        ROS_WARN_STREAM("Failed to init image_operator");
        return false;
    }

    return true;
}

void LineDetection::odometryCallback(const nav_msgs::OdometryConstPtr &odomMsg) {
	latestOdometry = *odomMsg;
}

void LineDetection::homographiedImageCallback(const sensor_msgs::ImageConstPtr &imgIn) {
    CvImagePtr currentImage = convertImageMessage(imgIn);
    processIncomingImage(*currentImage);
}

void LineDetection::processIncomingImage(cv::Mat &homographedImg) {
    imgHeight_ = homographedImg.rows;
    imgWidth_ = homographedImg.cols;
//    imgTimestamp = imgIn->header.stamp;
    imgTimestamp = ros::Time::now();

#ifdef PUBLISH_DEBUG
    cv::cvtColor(homographedImg, debugImg_, CV_GRAY2RGB);
#endif

    // Find lines in the image with Hough
    std::vector<Line> linesInImage;
    findLinesWithHough(homographedImg, linesInImage);

    if(linesInImage.empty()) {
        ROS_ERROR("No Hough lines found");
        return;
    }

#ifdef PUBLISH_DEBUG
    if(drawDebugLines_) {
        // Draw the Hough lines
        for(auto l : linesInImage) {
            cv::line(debugImg_, l.iP1_, l.iP2_, cv::Scalar(rand() % 255, rand() % 255, rand() % 255), 2);
        }

        // Display the given lane width
        std::vector<cv::Point2f> worldPts, imagePts;
        worldPts.push_back(cv::Point2f(0.2f, -.5f * laneWidthWorld_));
        worldPts.push_back(cv::Point2f(0.2f, .5f * laneWidthWorld_));

        image_operator_.worldToWarpedImg(worldPts, imagePts);

        cv::line(debugImg_, imagePts.at(0), imagePts.at(1), cv::Scalar(0,0,255), 4, cv::LINE_AA);
    }
#endif

    findLaneMarkings(linesInImage);

    // Publish the driving line message
    drive_ros_msgs::DrivingLine drivingLineMsg;
    Polynom drivingLinePoly, leftLinePoly, rightLinePoly;
    float detectionRange = roadModel.getDrivingLine(drivingLinePoly);
    bool leftLineFound = roadModel.getLaneMarkings(leftLinePoly, true);
    bool rightLineFound = roadModel.getLaneMarkings(rightLinePoly, false);

    drivingLineMsg.detectionRange = detectionRange;
    drivingLineMsg.polynom_order = drivingLinePoly.getOrder();

    for(auto c : drivingLinePoly.getCoeffs()) {
        drivingLineMsg.polynom_params.push_back(c);
    }

    // Left line polynom
    drivingLineMsg.left_line_found = leftLineFound;
    if(leftLineFound) {
      drivingLineMsg.left_poly_order = leftLinePoly.getOrder();
      for(auto c : leftLinePoly.getCoeffs()) {
        drivingLineMsg.left_poly_params.push_back(c);
      }
    }

    // Right line polynom
    drivingLineMsg.right_line_found = rightLineFound;
    if(rightLineFound) {
      drivingLineMsg.right_poly_order = rightLinePoly.getOrder();
      for(auto c : rightLinePoly.getCoeffs()) {
        drivingLineMsg.right_poly_params.push_back(c);
      }
    }

    drivingLinePub.publish(drivingLineMsg);

    // Check if we found an intersection
    std::vector<tf::Stamped<tf::Point>> intersectionPositions;
    std::vector<float> intersectionConfidences;
    std::vector<bool> intersectionStopLines;
    bool foundIntersections = roadModel.getIntersections(intersectionPositions, intersectionConfidences,
    		intersectionStopLines, drivingLinePoly);
    if(foundIntersections) {
    	drive_ros_msgs::DetectedIntersection intersectionMsg;
    	intersectionMsg.header.stamp = imgTimestamp;
    	for(int i = 0; i < intersectionPositions.size(); i++) {
    		geometry_msgs::PointStamped ptStamped;
    		tf::pointStampedTFToMsg(intersectionPositions.at(i), ptStamped);
    		intersectionMsg.positions.push_back(ptStamped);
    		intersectionMsg.confidences.push_back(intersectionConfidences.at(i));
    		intersectionMsg.stop_line_detected.push_back(intersectionStopLines.at(i));
    	}

    	// Publish the message
    	detectedIntersectionsPub.publish(intersectionMsg);
    }

#ifdef PUBLISH_DEBUG
    std::function<void (Polynom&, cv::Scalar)> drawLaneMarking = [this, detectionRange](Polynom &poly, cv::Scalar color) {
      std::vector<cv::Point2f> worldPts, imgPts;

      for(float x = 0.3f; x < detectionRange; x += .2f) {
          worldPts.push_back(cv::Point2f(x, poly.atX(x)));
      }
      worldPts.push_back(cv::Point2f(detectionRange, poly.atX(detectionRange)));

      image_operator_.worldToWarpedImg(worldPts, imgPts);

      for(int i = 1; i < imgPts.size(); i++) {
          cv::line(debugImg_, imgPts.at(i-1), imgPts.at(i), color, 2, cv::LINE_AA);
      }
    };

    drawLaneMarking(drivingLinePoly, cv::Scalar(0,255));
    if(leftLineFound) {
      drawLaneMarking(leftLinePoly, cv::Scalar(255,211,0));
    }
    if(rightLineFound) {
      drawLaneMarking(rightLinePoly, cv::Scalar(0,0,255));
    }

    // Draw intersections
    if(foundIntersections) {
    	std::vector<cv::Point2f> worldPts, imgPts;
    	for(int i = 0; i < intersectionPositions.size(); i++) {
    		worldPts.push_back(cv::Point2f(intersectionPositions.at(i).x(), intersectionPositions.at(i).y()));
    	}

    	if(image_operator_.worldToWarpedImg(worldPts, imgPts)) {
    		for(auto p : imgPts) {
    			cv::drawMarker(debugImg_, p, cv::Scalar(255), cv::MARKER_TRIANGLE_DOWN, 10, 3);
    		}
    	}
    }

    // Publish the debug image
    debugImgPub_.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, debugImg_).toImageMsg());
#endif

    //  auto t_start = std::chrono::high_resolution_clock::now();
    //  findLane();
    //  auto t_end = std::chrono::high_resolution_clock::now();
    //  ROS_INFO_STREAM("Cycle time: " << (std::chrono::duration<double, std::milli>(t_end-t_start).count()) << "ms");
}

void LineDetection::findLaneMarkings(std::vector<Line> &lines) {
    cv::Point2f segStartWorld(0.3f, 0.f);
    float segAngle = 0.f;
    float totalSegLength = 0.f;
    std::vector<Line*> unusedLines, leftMarkings, midMarkings, rightMarkings, verticalMarkings, otherMarkings;
    std::vector<Line*> allLeftMarkings, allMidMarkings, allRightMarkings;
    std::vector<cv::RotatedRect> regions;

    ROS_INFO("=============== New image ===============");

    for(int i = 0; i < lines.size(); i++) {
    	unusedLines.push_back(&(lines.at(i)));
    }

    // ================================
    // Find lane in new image
    // ================================
    std::vector<cv::Point2f> segmentStarts;
    std::vector<float> segmentAngles;
    int numSegmentsToUse = maxSenseRange_ / segmentLength_;
    bool useRoadModelSegment = true;
    roadModel.getSegmentPositions(segmentStarts, segmentAngles, imgTimestamp);

    for(int i = 0; i < numSegmentsToUse; i++) {
    	// clear the marking vectors. Otherwise the old ones stay in there.
    	leftMarkings.clear();
    	midMarkings.clear();
    	rightMarkings.clear();
    	verticalMarkings.clear();
    	otherMarkings.clear();

    	ROS_INFO("--- Segment #%u ---", i+1);

    	// If we did not find a segment at the previous position use one from the road model
    	if(useRoadModelSegment) {
    		if(i < segmentStarts.size()) {
    			segStartWorld = segmentStarts.at(i);
    			segAngle = segmentAngles.at(i);
    		}
    	}

    	std::vector<cv::RotatedRect> intersectionRegions;
        regions = buildRegions(intersectionRegions, segStartWorld, segAngle);
        assignLinesToRegions(&regions, segAngle, unusedLines, leftMarkings, midMarkings, rightMarkings, verticalMarkings, otherMarkings);

#if 0
        // DEBUG: draw the assigned lines
        cv::Mat testImg = debugImg_.clone();
        // first draw the regions
        for(auto r : regions) {
        	cv::Point2f edges[4];
        	r.points(edges);

        	for(int i = 0; i < 4; i++)
        		cv::line(testImg, edges[i], edges[(i+1)%4], cv::Scalar(255));
        }
        // draw the inliers
        std::function<void (std::vector<Line*>&, cv::Scalar)> drawLines =
        		[testImg](std::vector<Line*> &lines, cv::Scalar color) {
        	for(auto l : lines) {
        		cv::line(testImg, l->iP1_, l->iP2_, color, 2, cv::LINE_AA);
        	}
        };

        drawLines(leftMarkings, cv::Scalar(0,255));
        drawLines(midMarkings, cv::Scalar(0,255));
        drawLines(rightMarkings, cv::Scalar(0,255));
        drawLines(otherMarkings, cv::Scalar(0,0,255));

        std::string windowName("Assigned lines");
        cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE | cv::WINDOW_KEEPRATIO);
        cv::imshow(windowName, testImg);
        cv::waitKey(0);
#endif

        // DEBUG
        float probMidDashed = isDashedLine(midMarkings);
        float probLeftDashed = isDashedLine(leftMarkings);
        float probRightDashed = isDashedLine(rightMarkings);
        if((probMidDashed < probLeftDashed) || (probMidDashed < probRightDashed)) {
        	if(probMidDashed < 0.5f) {
        		ROS_WARN("  mid line is not dashed: %.2f", isDashedLine(midMarkings));
        		if(probRightDashed > 0.7f) {
        			ROS_INFO("  right line is probably the middle line: %.2f", isDashedLine(rightMarkings));

        			cv::Vec2f shiftVec(	sin(segAngle) * laneWidthWorld_,
        					cos(segAngle) * laneWidthWorld_ * -1.f);

        			useRoadModelSegment = false;
        			i--; // redo this segment
        			segStartWorld.x += shiftVec(0);
        			segStartWorld.y += shiftVec(1);
        			continue;
        		} else if(probLeftDashed > 0.7f) {
        			ROS_INFO("  left line is probably the middle line: %.2f", isDashedLine(leftMarkings));

        			cv::Vec2f shiftVec(	sin(segAngle) * laneWidthWorld_,
        					cos(segAngle) * laneWidthWorld_);

        			useRoadModelSegment = false;
        			i--; // redo this segment
        			segStartWorld.x += shiftVec(0);
        			segStartWorld.y += shiftVec(1);
        			continue;
        		} else {
        			ROS_INFO("!!! Maybe there are markings on the street?");
        		}
        	}
        }

        // Find the street segment with RANSAC
        float intersectionDistanceLines, intersectionDistanceBasedOnAngle;
        bool intersectionBasedOnAngle;
        bool intersectionWithStopLine; // This could be a stop or yield line
        auto seg = findLaneWithRansac(leftMarkings, midMarkings, rightMarkings, segStartWorld, segAngle, i == 0);
        bool intersectionBasedOnLines = findIntersection(&intersectionRegions, segAngle, segStartWorld,
        		verticalMarkings, intersectionDistanceLines, intersectionWithStopLine);

        // Check if the found segment fits with the previous one
        if(roadModel.segmentFitsToPrevious(&seg, i, intersectionBasedOnAngle)) {
        	seg.creationTimestamp = imgTimestamp;
        	roadModel.updateSegmentAtIndex(seg, i);

        	segAngle = seg.angleTotal; // the current angle of the lane
        	totalSegLength += seg.length;
        	segStartWorld = seg.endPositionWorld;
        	useRoadModelSegment = false;

        	allLeftMarkings.insert(allLeftMarkings.end(), leftMarkings.begin(), leftMarkings.end());
        	allMidMarkings.insert(allMidMarkings.end(), midMarkings.begin(), midMarkings.end());
        	allRightMarkings.insert(allRightMarkings.end(), rightMarkings.begin(), rightMarkings.end());
        } else {
        	ROS_INFO("  segment does not fit with previous");
        	useRoadModelSegment = true;
        	roadModel.decreaseSegmentTtl(i);

        	if(intersectionBasedOnAngle) {
        		intersectionDistanceBasedOnAngle = segStartWorld.x;
        	}
        }

        // check if we found an intersection
        if(intersectionBasedOnAngle && intersectionBasedOnLines) {
        	if(fabsf(intersectionDistanceBasedOnAngle - intersectionDistanceLines) < segmentLength_) {
        		roadModel.addIntersectionAt(intersectionDistanceLines, 1.f, intersectionWithStopLine);
        	} else {
        		ROS_WARN("  Found intersection twice, but not at same position");
        	}
        } else if(intersectionBasedOnAngle) {
        	// If our find intersection did not find anything, it is pretty sure there was no stop line
        	roadModel.addIntersectionAt(intersectionDistanceBasedOnAngle, .5f, false);
        } else if(intersectionBasedOnLines) {
        	roadModel.addIntersectionAt(intersectionDistanceLines, .5f, intersectionWithStopLine);
        }
    }

//    ROS_INFO("  All left markings: %.3f", isDashedLine(allLeftMarkings));
//    ROS_INFO("  All mid markings: %.3f", isDashedLine(allMidMarkings));
//    ROS_INFO("  All right markings: %.3f", isDashedLine(allRightMarkings));

    roadModel.setOdomPointsForSegments();
    roadModel.decreaseAllSegmentTtl();

    // DEBUG
    std::vector<cv::Point2f> worldPts, imgPts;
    std::vector<cv::Scalar> colors;
    std::vector<float> angles;

    roadModel.getSegmentPositions(worldPts, angles, ros::Time(0));
    for(auto s : roadModel.segmentsToDl)  {
        if(s.ttl == 3) {
            colors.push_back(cv::Scalar(0,0,255));
        } else if(s.ttl > 0){
            colors.push_back(cv::Scalar(255));
        } else {
            colors.push_back(cv::Scalar(0,0,0));
        }
    }

    if (worldPts.size() > 0)
        image_operator_.worldToWarpedImg(worldPts, imgPts);
    else
        ROS_WARN("[Line Detection] No segment points found in image!");

    for(int i = 0; i < imgPts.size(); i++) {
        cv::circle(debugImg_, imgPts.at(i), 5, colors.at(i), 3);
    }
}

///
/// \brief LineDetection::buildRegions
/// \param position Position of the segment in world coordinates
/// \param angle orientation of the segment in [rad]
/// \return the three regions in image coordinates
///
std::vector<cv::RotatedRect> LineDetection::buildRegions(
		std::vector<cv::RotatedRect> &intersectionRegions, cv::Point2f positionWorld, float angle) {
	int numRegions = 5; // 3 line search segment + 2 intersection
    std::vector<cv::RotatedRect> regions(numRegions);
    std::vector<cv::Point2f> imgPts(1), worldPts(3);
    cv::Vec2f dirVec(cos(angle), sin(angle)); // in world coordinates
    cv::Vec2f leftVec(laneWidthWorld_ * dirVec[1], laneWidthWorld_ * dirVec[0]); // in world coordinates
    intersectionRegions.clear();

    // Convert the position to image coordinates
    worldPts.at(0) = positionWorld;
    image_operator_.worldToWarpedImg(worldPts, imgPts);
    cv::Point2f positionImg = imgPts.at(0);

    // Calculate the region size in world coordinates and then convert to size in image coordinates
    cv::Point2f worldPtOuterRegionSize(positionWorld.x + segmentLength_, positionWorld.y + (1.2f * laneWidthWorld_));
    worldPts.at(0) = worldPtOuterRegionSize;
    cv::Point2f worldPtMiddleRegionSize(positionWorld.x + segmentLength_, positionWorld.y + (1.f * laneWidthWorld_));
    worldPts.at(1) = worldPtMiddleRegionSize;
    cv::Point2f worldPtIntersectionMidSize(positionWorld.x + segmentLength_, positionWorld.y +(1.5f * laneWidthWorld_));
    worldPts.at(2) = worldPtIntersectionMidSize;
    image_operator_.worldToWarpedImg(worldPts, imgPts);
    cv::Size outerRegionSize(imgPts.at(0).x - positionImg.x, positionImg.y - imgPts.at(0).y);
    cv::Size innerRegionSize(imgPts.at(1).x - positionImg.x, positionImg.y - imgPts.at(1).y);
    cv::Size innerIntersectionRegionSize(imgPts.at(2).x - positionImg.x, positionImg.y - imgPts.at(2).y);

    // Get the region points in world coordinates
    worldPts.resize(numRegions);
    imgPts.resize(numRegions);

    cv::Point2f centerMidRegion(positionWorld.x + (0.5f * segmentLength_ * dirVec[0]) - (0.5f * leftVec[0]),
         	    				positionWorld.y + (0.5f * segmentLength_ * dirVec[1]) + (0.5f * leftVec[1]));

    cv::Point2f centerRightRegion(centerMidRegion.x + 1.1f*leftVec[0],
    							  centerMidRegion.y - 1.1f*leftVec[1]);

    cv::Point2f centerLeftRegion(centerMidRegion.x - 1.1f*leftVec[0],
    							 centerMidRegion.y + 1.1f*leftVec[1]);

    cv::Point2f centerRightIntersectionRegion(centerMidRegion.x + 1.8f*leftVec[0],
    										  centerMidRegion.y - 1.8f*leftVec[1]);

    cv::Point2f centerLeftIntersectionRegion(centerMidRegion.x - 1.8f*leftVec[0],
    										 centerMidRegion.y + 1.8f*leftVec[1]);

    worldPts.at(0) = centerLeftRegion;
    worldPts.at(1) = centerMidRegion;
    worldPts.at(2) = centerRightRegion;
    worldPts.at(3) = centerRightIntersectionRegion;
    worldPts.at(4) = centerLeftIntersectionRegion;
    image_operator_.worldToWarpedImg(worldPts, imgPts);

    // Create the regions
    regions.at(0) = cv::RotatedRect(imgPts.at(0), outerRegionSize, (-angle * 180.f / M_PI));
    regions.at(1) = cv::RotatedRect(imgPts.at(1), innerRegionSize, (-angle * 180.f / M_PI));
    regions.at(2) = cv::RotatedRect(imgPts.at(2), outerRegionSize, (-angle * 180.f / M_PI));

    // Build the intersection regions
    intersectionRegions.push_back(cv::RotatedRect(imgPts.at(4), innerRegionSize, (-angle * 180.f / M_PI)));
    intersectionRegions.push_back(cv::RotatedRect(imgPts.at(1), innerIntersectionRegionSize, (-angle * 180.f / M_PI)));
    intersectionRegions.push_back(cv::RotatedRect(imgPts.at(3), innerRegionSize, (-angle * 180.f / M_PI)));

#ifdef PUBLISH_DEBUG
    if(drawDebugLines_) {
    	for(int i = 0; i < 3; i++) {
    		auto r = regions.at(i);
    		cv::Point2f edges[4];
    		r.points(edges);

    		for(int i = 0; i < 4; i++)
    			cv::line(debugImg_, edges[i], edges[(i+1)%4], cv::Scalar(255));
    	}
    }
#endif

#if 0
    for(int i = 0; i < 3; i++) {
    	auto r = intersectionRegions.at(i);
    	cv::Point2f edges[4];
    	r.points(edges);

    	for(int i = 0; i < 4; i++)
    		cv::line(debugImg_, edges[i], edges[(i+1)%4], cv::Scalar(255,211), 2);
    }
#endif

    return regions;
}

///
/// \brief LineDetection::assignLinesToRegions assigns all lines to one of the three regions (left, middle, right)
/// or to others.
/// \param regions three regions (left, middle, right)
/// \param lines the lines which should be assigned to the regions
///
void LineDetection::assignLinesToRegions(std::vector<cv::RotatedRect> *regions, float angle, std::vector<Line*> &lines,
                                         std::vector<Line*> &leftMarkings, std::vector<Line*> &midMarkings,
                                         std::vector<Line*> &rightMarkings, std::vector<Line*> &verticalMarkings,
										 std::vector<Line*> &otherMarkings) {

	for(auto linesIt = lines.begin(); linesIt != lines.end(); ++linesIt) {
		if(lineIsInRegion(*linesIt, &(regions->at(0)), true)) {
			if(fabsf(angle - (*linesIt)->getAngle()) > M_PI_4)
				verticalMarkings.push_back(*linesIt);
			else
				leftMarkings.push_back(*linesIt);
		} else if(lineIsInRegion(*linesIt, &(regions->at(1)), true)) {
			if(fabsf(angle - (*linesIt)->getAngle()) > M_PI_4)
				verticalMarkings.push_back(*linesIt);
			else
				midMarkings.push_back(*linesIt);
		} else if(lineIsInRegion(*linesIt, &(regions->at(2)), true)) {
			if(fabsf(angle - (*linesIt)->getAngle()) > M_PI_4)
				verticalMarkings.push_back(*linesIt);
			else
				rightMarkings.push_back(*linesIt);
		} else {
			otherMarkings.push_back(*linesIt);
		}
	}
}

float LineDetection::isDashedLine(std::vector<Line*> &laneMarkings) {
	int numNotDashed = 0;
//	ROS_INFO("  isDashedLine: num lane markings: %lu", laneMarkings.size());

	if(laneMarkings.size() < 3) {
		return .51f; // TODO: put lower bound in config1
	}

	std::sort(laneMarkings.begin(), laneMarkings.end(), [](Line *a, Line *b){ return a->wP1_.x < b->wP1_.x; });

	for(auto i = laneMarkings.begin(); i != laneMarkings.end(); ++i) {
		// Line should not be longer than 0.2[m]
		if((*i)->getWorldLength() > 0.21f) {
			numNotDashed++;
			continue; // next line for iterator 'i'
		}

		// There should be no line closer than 0.2[m] to this line's end
		for(auto j = i + 1; j != laneMarkings.end(); ++j) {
			if((*i)->wP2_.x < (*j)->wP1_.x) {
				auto xDist = (*j)->wP1_.x - (*i)->wP2_.x;
				auto yDist = (*j)->wP1_.y - (*i)->wP2_.y;

				if(sqrtf(xDist*xDist + yDist*yDist) < 0.19f) {
					numNotDashed++;
					break; // continue with next 'i' since we already found a line closer than the threshold
				}
			}
		}
	}

	return 1.f - (float(numNotDashed) / float(laneMarkings.size()));
}

///
/// \brief LineDetection::lineIsInRegion
/// \param line
/// \param region in image coordinates
/// \param isImageCoordinate true, if region is given in image coordinates
/// \return
///
bool LineDetection::lineIsInRegion(Line *line, const cv::RotatedRect *region, bool isImageCoordiante) const {
    cv::Point2f edges[4];
    region->points(edges); // bottomLeft, topLeft, topRight, bottomRight

    if(isImageCoordiante) {
    	// we check if at least on of the lines ends is in the region
    	if(pointIsInRegion(&(line->iP1_), edges))
    		return true;
        else if(pointIsInRegion(&(line->iP2_), edges))
            return true;
        else {
            // in case the line start and end are outside the rect, the line can still intersect with it
            std::vector<cv::Point2f> pts;
            pts.push_back(line->iP1_);
            pts.push_back(line->iP2_);
            auto lineRect = cv::minAreaRect(pts);
            lineRect.size.height += 1;
            lineRect.size.width += 1;
//            ROS_INFO_STREAM("Points " << line->iP1_ << " and " << line->iP2_ << " give Rect at " <<
//            		lineRect.center << " with size " << lineRect.size);
            int res = cv::rotatedRectangleIntersection(lineRect, *region, pts);
            if(res == cv::INTERSECT_NONE) {
                return false;
            } else {
                return true;
            }
        }

    } else {
    	// we check if at least on of the lines ends is in the region
    	if(pointIsInRegion(&(line->wP1_), edges))
    		return true;
    	else
    		return pointIsInRegion(&(line->wP2_), edges);
    }
}

bool LineDetection::pointIsInRegion(cv::Point2f *pt, cv::Point2f *edges) const {
    auto u = edges[0] - edges[1];
    auto v = edges[0] - edges[3];

    auto uDotPt = u.dot(*pt);
    auto uDotP1 = u.dot(edges[0]);
    auto uDotP2 = u.dot(edges[1]);
    auto vDotPt = v.dot(*pt);
    auto vDotP1 = v.dot(edges[0]);
    auto vDotP4 = v.dot(edges[3]);

    bool xOk = (uDotPt < uDotP1 && uDotPt > uDotP2) ||
               (uDotPt > uDotP1 && uDotPt < uDotP2);
    bool yOk = (vDotPt < vDotP1 && vDotPt > vDotP4) ||
               (vDotPt > vDotP1 && vDotPt < vDotP4);

    return xOk && yOk;
}

///
/// \brief LineDetection::findLaneWithRansac
/// \param leftMarkings
/// \param midMarkings
/// \param rightMarkings
/// \param segStartWorld the segments position in world coordinates
/// \param prevAngle
/// \return
///
Segment LineDetection::findLaneWithRansac(std::vector<Line*> &leftMarkings,
                                          std::vector<Line*> &midMarkings,
                                          std::vector<Line*> &rightMarkings,
                                          cv::Point2f segStartWorld, float prevAngle, bool isFirstSegment) {
    float bestAngle = prevAngle;
    float bestScore = 0.f;
    Line *bestLine;
    cv::Point2f laneMidPt = segStartWorld;
    cv::Point2f bestLeft, bestMid, bestRight;
    size_t numLines = leftMarkings.size() + midMarkings.size() + rightMarkings.size();
    size_t iteration = 0;
    std::vector<cv::Point2f> worldPts, imgPts;
    worldPts.push_back(segStartWorld);
    image_operator_.worldToWarpedImg(worldPts, imgPts);

    if(numLines == 0) {
        ROS_WARN("  no lines for Ransac");
        return Segment(segStartWorld, imgPts.at(0), 0.f, prevAngle, segmentLength_, 0.f);
    }

    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0, numLines - 1);

//    ROS_INFO_STREAM("Left markings: " << leftMarkings.size());
//    ROS_INFO_STREAM("Mid markings: " << midMarkings.size());
//    ROS_INFO_STREAM("Right markings: " << rightMarkings.size());

    while(bestScore < 0.9 && (iteration++ < maxRansacInterations_)) {
        // Select a random line and get its angle
    	// Also get a random point and move it on the expected middle line
        int randomIdx = distribution(generator);
        float currentAngle = prevAngle;
        Line *currentLine;

        // 1) Select a random line
        if(randomIdx < leftMarkings.size()) {
        	currentLine = leftMarkings.at(randomIdx);
        } else if((randomIdx - leftMarkings.size()) < midMarkings.size()) {
        	currentLine = midMarkings.at(randomIdx - leftMarkings.size());
        } else {
        	currentLine = rightMarkings.at(randomIdx - leftMarkings.size() - midMarkings.size());
        }

        currentAngle = currentLine->getAngle();

        if(fabsf(currentAngle - prevAngle) > M_PI_2) {
        	continue;
        }

        // 2) Test how many lines have about the same angle and their distance fits
        int numInliers = 0;

        std::function<bool (Line*)> lineCmpFunc = [this, currentAngle, currentLine](Line *l) {
          if(fabsf(l->getAngle() - currentAngle) > (5.f / 180.f * M_PI)) {
            // angle diff should be < 5 [degree]
            return false;
          }

          float distanceToLine = distanceBetweenLines(*l, *currentLine);

          if(distanceToLine < laneVar_) {
            return true;
          }

          if(fabsf(distanceToLine - laneWidthWorld_) < laneVar_) {
            return true;
          }

          if(fabsf(distanceToLine - 2*laneWidthWorld_) < laneVar_) {
            return true;
          }

          return false;
        };

        numInliers += std::count_if(leftMarkings.begin(), leftMarkings.end(), lineCmpFunc);
        numInliers += std::count_if(midMarkings.begin(), midMarkings.end(), lineCmpFunc);
        numInliers += std::count_if(rightMarkings.begin(), rightMarkings.end(), lineCmpFunc);

        // 3) Compute score
        float newScore = static_cast<float>(numInliers) / static_cast<float>(numLines);
        if(newScore > bestScore) {
            bestScore = newScore;
            bestAngle = currentAngle;
            bestLine = currentLine;
        }
    }

    // 4) Collect points from every lane marking
    std::function<bool (Line*)> inlierFunc = [this, bestAngle, bestLine](Line *l) {
    	if(fabsf(l->getAngle() - bestAngle) > (5.f / 180.f * M_PI)) {
    		// angle diff should be < 5 [degree]
    		return false;
    	}

    	float distanceToLine = distanceBetweenLines(*l, *bestLine);

    	if(distanceToLine < laneVar_) {
    		return true;
    	}

    	if(fabsf(distanceToLine - laneWidthWorld_) < laneVar_) {
    		return true;
    	}

    	if(fabsf(distanceToLine - 2*laneWidthWorld_) < (2*laneVar_)) {
    		return true;
    	}

    	return false;

    };

    std::vector<cv::Point2f> leftInlierPtsWorld, midInlierPtsWorld, rightInlierPtsWorld;

    for(auto l : midMarkings) {
    	if(inlierFunc(l)) {
//    		cv::line(debugImg_, l->iP1_, l->iP2_, cv::Scalar(0,255), 3, cv::LINE_AA);
    		midInlierPtsWorld.push_back(l->wP1_);
    		midInlierPtsWorld.push_back(l->wP2_);
    	}
    }

    for(auto l : leftMarkings) {
    	if(inlierFunc(l)) {
//    		cv::line(debugImg_, l->iP1_, l->iP2_, cv::Scalar(0,0,255), 3, cv::LINE_AA);
    		leftInlierPtsWorld.push_back(l->wP1_);
    		leftInlierPtsWorld.push_back(l->wP2_);
    	}
    }

    for(auto l : rightMarkings) {
    	if(inlierFunc(l)) {
//    		cv::line(debugImg_, l->iP1_, l->iP2_, cv::Scalar(0,0,255), 3, cv::LINE_AA);
    		rightInlierPtsWorld.push_back(l->wP1_);
    		rightInlierPtsWorld.push_back(l->wP2_);
    	}
    }

//    ROS_INFO("  Left inliers: %lu out of %lu", leftInlierPtsWorld.size() / 2, leftMarkings.size());
//    ROS_INFO("  Mid inliers: %lu out of %lu", midInlierPtsWorld.size() / 2, midMarkings.size());
//    ROS_INFO("  Right inliers: %lu out of %lu", rightInlierPtsWorld.size() / 2, rightMarkings.size());

    // 5) Fit polynom in each line
    int lanePosFrom = 0; // DEBUG
    float detectedRange = 0.f;
    float detectedLen = .0f;
    int numLaneMarkingsDetected = 0;
    bool leftLineFound = false, rightLineFound = false;
    cv::Point2f leftLineStart, leftLineEnd, rightLineStart, rightLineEnd;

    if(!leftInlierPtsWorld.empty()) {
    	Polynom poly(1, leftInlierPtsWorld);

    	worldPts.clear();
    	imgPts.clear();

    	auto minElem = std::min_element(leftInlierPtsWorld.begin(), leftInlierPtsWorld.end(), [](cv::Point2f &a, cv::Point2f &b) {
    		return a.x < b.x;
    	});
    	auto maxElem = std::max_element(leftInlierPtsWorld.begin(), leftInlierPtsWorld.end(), [](cv::Point2f &a, cv::Point2f &b) {
    		return a.x < b.x;
    	});

    	float xMin = minElem->x;
    	if(isFirstSegment) {
    		xMin = std::min(xMin, .3f);
    	}
    	float xMax = maxElem->x;

    	// Set lane position
    	laneMidPt = cv::Point2f(segStartWorld.x, poly.atX(segStartWorld.x));
    	cv::Vec2f shiftVec(sin(bestAngle) * laneWidthWorld_ * 1.5f, cos(bestAngle) * laneWidthWorld_ * -1.5f);
    	laneMidPt.x += shiftVec(0);
    	laneMidPt.y += shiftVec(1);
    	lanePosFrom = 1; // Debug

    	detectedRange += xMax;
    	numLaneMarkingsDetected++;

    	// Set start and end of line
    	leftLineFound = true;
    	leftLineStart.x = xMin;
    	leftLineStart.y = poly.atX(xMin);
    	leftLineEnd.x = xMax;
    	leftLineEnd.y = poly.atX(xMax);

    	// Calculate detected length
    	detectedLen += getDistanceBetweenPoints(leftLineStart, leftLineEnd);
    }

    if(!rightInlierPtsWorld.empty()) {
    	Polynom poly(1, rightInlierPtsWorld);

    	worldPts.clear();
    	imgPts.clear();

    	auto minElem = std::min_element(rightInlierPtsWorld.begin(), rightInlierPtsWorld.end(), [](cv::Point2f &a, cv::Point2f &b) {
    		return a.x < b.x;
    	});
    	auto maxElem = std::max_element(rightInlierPtsWorld.begin(), rightInlierPtsWorld.end(), [](cv::Point2f &a, cv::Point2f &b) {
    		return a.x < b.x;
    	});

    	float xMin = minElem->x;
    	if(isFirstSegment) {
    		xMin = std::min(xMin, .3f);
    	}
    	float xMax = maxElem->x;

    	// Set lane position
    	laneMidPt = cv::Point2f(segStartWorld.x, poly.atX(segStartWorld.x));
    	cv::Vec2f shiftVec(sin(bestAngle) * laneWidthWorld_ * .5f, cos(bestAngle) * laneWidthWorld_ * .5f);
    	laneMidPt.x += shiftVec(0);
    	laneMidPt.y += shiftVec(1);
    	lanePosFrom = 3;

    	detectedRange += xMax;
    	numLaneMarkingsDetected++;

    	// Set start and end of line
    	rightLineFound = true;
    	rightLineStart.x = xMin;
    	rightLineStart.y = poly.atX(xMin);
    	rightLineEnd.x = xMax;
    	rightLineEnd.y = poly.atX(xMax);

    	// Calculate detected length
    	detectedLen += getDistanceBetweenPoints(rightLineStart, rightLineEnd);
    }

    if(!midInlierPtsWorld.empty()) {
    	Polynom poly(1, midInlierPtsWorld);

    	worldPts.clear();
    	imgPts.clear();

    	auto minElem = std::min_element(midInlierPtsWorld.begin(), midInlierPtsWorld.end(), [](cv::Point2f &a, cv::Point2f &b) {
    		return a.x < b.x;
    	});
    	auto maxElem = std::max_element(midInlierPtsWorld.begin(), midInlierPtsWorld.end(), [](cv::Point2f &a, cv::Point2f &b) {
    		return a.x < b.x;
    	});

    	float xMin = minElem->x;
    	float xMax = maxElem->x;

    	// Set lane position
    	laneMidPt = cv::Point2f(segStartWorld.x, poly.atX(segStartWorld.x));
    	cv::Vec2f shiftVec(sin(bestAngle) * laneWidthWorld_ * .5f, cos(bestAngle) * laneWidthWorld_ * -.5f);
    	laneMidPt.x += shiftVec(0);
    	laneMidPt.y += shiftVec(1);
    	lanePosFrom = 2;

    	detectedRange += xMax;
    	numLaneMarkingsDetected++;

    	// Calculate detected length
    	cv::Point2f midStart, midEnd;
    	midStart.x = xMin;
    	midStart.y = poly.atX(xMin);
    	midEnd.x = xMax;
    	midEnd.y = poly.atX(xMax);
    	detectedLen += getDistanceBetweenPoints(midStart, midEnd);
    }


    // 6) Build the segment
    // Calculate the segment end point
    detectedRange /= numLaneMarkingsDetected;
    detectedLen /= numLaneMarkingsDetected;
    float segmentLen = std::min(detectedLen, segmentLength_);
    cv::Point2f segEndWorld;
    segEndWorld.x = laneMidPt.x + cos(bestAngle) * segmentLen;
    segEndWorld.y = laneMidPt.y + sin(bestAngle) * segmentLen;

    // Convert all needed points to image coordinates
    worldPts.clear();
    imgPts.clear();
    worldPts.push_back(laneMidPt); // idx 0
    worldPts.push_back(bestLeft); // idx 1
    worldPts.push_back(bestMid); // idx 2
    worldPts.push_back(bestRight); // idx 3
    worldPts.push_back(segEndWorld); // idx 4
    image_operator_.worldToWarpedImg(worldPts, imgPts);

    // Create the segment
    Segment s(laneMidPt, imgPts.at(0), prevAngle - bestAngle, bestAngle, segmentLen, bestScore);
    s.leftPosW = bestLeft;
    s.midPosW = bestMid;
    s.rightPosW = bestRight;
    s.leftPosI = imgPts.at(1);
    s.midPosI = imgPts.at(2);
    s.rightPosI = imgPts.at(3);
    s.endPositionImage = imgPts.at(4);
    s.endPositionWorld = segEndWorld;
    s.leftLineSet = leftLineFound;
    if(leftLineFound) {
    	s.leftLineStart = leftLineStart;
    	s.leftLineEnd = leftLineEnd;
    }
    s.rightLineSet = rightLineFound;
    if(rightLineFound) {
    	s.rightLineStart = rightLineStart;
    	s.rightLineEnd = rightLineEnd;
    }

//    ROS_INFO("  Lane position built from %s line", ((lanePosFrom == 1) ? "left" : ((lanePosFrom == 2) ? "mid" : "right")));

    return s;
}

bool LineDetection::findIntersection(std::vector<cv::RotatedRect> *regions, float segmentAngle, cv::Point2f segStartWorld,
		std::vector<Line*> &verticalMarkings, float &distanceToIntersection, bool &stopLineFound) {

	bool foundIntersection = false;
	bool middleExists = false, rightExists = false, leftExists = false;
	float stopLineXpos = 0.f;
	stopLineFound = false;

	// Assign the lines to the regions (left, middle, right)
	std::vector<Line*> leftMarkings, midMarkings, rightMarkings;
	for(auto linesIt = verticalMarkings.begin(); linesIt != verticalMarkings.end(); ++linesIt) {
		// first check the angle
		if((fabsf((*linesIt)->getAngle() - segmentAngle) - M_PI_2) < (5.f / 180.f * M_PI)) {
			// Ignore lines closer than 0.2[m] because these could be from the car
			if((*linesIt)->wP1_.x < .2f || (*linesIt)->wP2_.x < .2f) {
				continue;
			}

			// A line can lay in more than one region
			if(lineIsInRegion(*linesIt, &(regions->at(0)), true)) {
				leftMarkings.push_back(*linesIt);
			}
			if(lineIsInRegion(*linesIt, &(regions->at(1)), true)) {
				midMarkings.push_back(*linesIt);
			}
			if(lineIsInRegion(*linesIt, &(regions->at(2)), true)) {
				rightMarkings.push_back(*linesIt);
			}
		}
	}

	leftExists = leftMarkings.size() > 1;
	middleExists = midMarkings.size() > 1;
	rightExists = rightMarkings.size() > 1;

	ROS_INFO("  Left: %lu  Mid: %lu  Right: %lu", leftMarkings.size(), midMarkings.size(), rightMarkings.size());

	// if there think there is an intersection, we calculate the distance to it
	if(middleExists) {
		for(auto l : midMarkings) {
			stopLineXpos += l->wP1_.x;
			stopLineXpos += l->wP2_.x;
		}
	}

	distanceToIntersection = stopLineXpos / (2 * midMarkings.size());

	// guess the type of the intersection (stop/yield, go-through, start line)
	if(distanceToIntersection < .3f) {
		// We sometimes get vertical lines from the car or at the warped image boarder
		return false;
	}

	if(middleExists && (leftExists || rightExists)) {
		ROS_INFO("  INTERSECTION found - stop in %.2f[m]", distanceToIntersection);
		foundIntersection = true;
		stopLineFound = true;
	}
	if(!middleExists && leftExists && rightExists) {
		distanceToIntersection = segStartWorld.x;
		ROS_INFO("  INTERSECTION found - do not stop in %.2f[m]", distanceToIntersection);
		foundIntersection = true;
	}
	if(middleExists && !leftExists && !rightExists) {
		ROS_INFO("  START LINE found");
	}

	return foundIntersection;
}

///
/// \brief LineDetection::findLinesWithHough
/// Extract Lines from the image using Hough Lines
/// \param img A CvImagePtr to the current image where we want to search for lines
/// \param houghLines A std::vector where the Lines will be returned. Has to be empty.
///
void LineDetection::findLinesWithHough(cv::Mat &img, std::vector<Line> &houghLines) {
    cv::Mat processingImg;

    // Blur the image and find edges with Canny
    cv::GaussianBlur(img, processingImg, cv::Size(15, 15), 0, 0);
    cv::Canny(processingImg, processingImg, cannyThreshold_, cannyThreshold_ * 3, 3);


    // Get houghlines
    std::vector<cv::Vec4i> hLinePoints;
    cv::HoughLinesP(processingImg, hLinePoints, 1, CV_PI / 180, houghThresold_, houghMinLineLen_, houghMaxLineGap_);

    // Extract points for houghLines and convert to world-coordinates
    std::vector<cv::Point2f> imagePoints, worldPoints;
    for(size_t i = 0; i < hLinePoints.size(); i++) {
        cv::Vec4i currentPoints = hLinePoints.at(i);
        // Ignore lines very close to the car and about to be parallel to image y-axis
        if(!((currentPoints[0] < 50) && (currentPoints[2] < 50))) {
            imagePoints.push_back(cv::Point(currentPoints[0], currentPoints[1]));
            imagePoints.push_back(cv::Point(currentPoints[2], currentPoints[3]));
        }
    }

    if (imagePoints.size() == 0) {
        ROS_WARN_STREAM("No hough lines found in image");
        return;
    }
    image_operator_.warpedImgToWorld(imagePoints, worldPoints);

    // Build lines from points
    std::vector<Line> lines;
    for(size_t i = 0; i < worldPoints.size(); i += 2) {
        houghLines.push_back(Line(imagePoints.at(i), imagePoints.at(i + 1), worldPoints.at(i), worldPoints.at(i + 1)));
    }
}

/*
 * In world coordinates
 */
float LineDetection::pointToLineDistance(Line &l, const cv::Point2f &p) {
    // http://paulbourke.net/geometry/pointlineplane/

    if(getDistanceBetweenPoints(l.wP1_, l.wP2_) == 0) {
        ROS_WARN("Line with length 0");
        return std::numeric_limits<float>::max();
    }

    float u = ((p.x - l.wP1_.x)*(l.wP2_.x - l.wP1_.x) + (p.y - l.wP1_.y)*(l.wP2_.y - l.wP1_.y)) /
            (getDistanceBetweenPoints(l.wP1_, l.wP2_));

    float intersectionPointX = l.wP1_.x + u*(l.wP2_.x - l.wP1_.x);
    float intersectionPointY = l.wP1_.y + u*(l.wP2_.y - l.wP1_.y);

    return getDistanceBetweenPoints(p, cv::Point2f(intersectionPointX, intersectionPointY));
}

float LineDetection::distanceBetweenLines(Line &a, Line &b) {
    return	fminf(pointToLineDistance(a, b.wP1_),
                  fminf(pointToLineDistance(a, b.wP2_),
                        fminf(pointToLineDistance(b, a.wP1_), pointToLineDistance(b, a.wP2_))));
}

///
/// \brief LineDetection::reconfigureCB
/// Used by the dynamic reconfigure server
/// \param config
/// \param level
///
void LineDetection::reconfigureCB(drive_ros_image_recognition::LineDetectionConfig& config, uint32_t level) {
    image_operator_.setConfig(config);
    drawDebugLines_ = config.draw_debug;
    laneWidthWorld_ = config.lane_width;
    laneVar_ = config.lane_var;
    lineAngle_ = config.lineAngle;
    maxSenseRange_ = config.maxSenseRange;
    cannyThreshold_ = config.cannyThreshold;
    houghThresold_ = config.houghThreshold;
    houghMinLineLen_ = config.houghMinLineLen;
    houghMaxLineGap_ = config.houghMaxLineGap;
    segmentLength_ = config.segmentLength;
    maxRansacInterations_ = config.ransacIterations;

    roadModel.setLaneWidth(laneWidthWorld_);
    roadModel.setDefaultPolyOrder(config.poly_order);
    roadModel.setMaxPolyErrorThresh(config.poly_error_thresh);
}

} // namespace drive_ros_image_recognition
