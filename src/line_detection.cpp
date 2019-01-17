#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <chrono>
#include <pluginlib/class_list_macros.h>
#include <eigen3/Eigen/Dense>
#include <random>
#include "drive_ros_image_recognition/line_detection.h"
#include "drive_ros_image_recognition/geometry_common.h"
#include "drive_ros_msgs/RoadLine.h"
#include "drive_ros_msgs/simple_trajectory.h"
#include "drive_ros_msgs/DrivingLine.h"

namespace drive_ros_image_recognition {

LineDetection::LineDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh)
    : nh_(nh)
    , pnh_(pnh)
    , imageTransport_(pnh)
    , laneWidthWorld_(0.4)
	, lineVar_(0.1)
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

    //subscribe to camera image
    imageSubscriber_ = imageTransport_.subscribe("/img_in", 3, &LineDetection::imageCallback, this);
    ROS_INFO_STREAM("Subscribed image transport to topic " << imageSubscriber_.getTopic());

    odometrySub = pnh_.subscribe("odom_topic", 3, &LineDetection::odometryCallback, this);
    ROS_INFO("Subscribing to odometry on topic '%s'", odometrySub.getTopic().c_str());

    drivingLinePub = nh_.advertise<drive_ros_msgs::DrivingLine>("driving_line_topic", 1);
    ROS_INFO("Publish driving line on topic '%s'", drivingLinePub.getTopic().c_str());

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

///
/// \brief LineDetection::imageCallback
/// Called for incoming camera image. Extracts the image from the incoming message.
/// \param imgIn the incoming camera image message.
///
void LineDetection::imageCallback(const sensor_msgs::ImageConstPtr &imgIn) {
    // TODO: so far these states are only for the free drive (w/o obstacles) and probably not even complete, yet
    auto currentImage = convertImageMessage(imgIn);
    cv::Mat homographedImg;
    if(!image_operator_.homographImage(*currentImage, homographedImg)) {
        ROS_WARN("Homographing image failed");
        return;
    }
    imgHeight_ = homographedImg.rows;
    imgWidth_ = homographedImg.cols;
//    imgTimestamp = imgIn->header.stamp;
    imgTimestamp = ros::Time::now();

#ifdef PUBLISH_DEBUG
    cv::cvtColor(homographedImg, debugImg_, CV_GRAY2RGB);
#endif

    std::vector<Line> linesInImage;
    findLinesWithHough(homographedImg, linesInImage);

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

    findLaneMarkings(linesInImage);

    drive_ros_msgs::DrivingLine drivingLineMsg;
    auto dl = roadModel.getDrivingLine();
    drivingLineMsg.detectionRange = dl.detectionRange;
    drivingLineMsg.polynom_order = dl.poly.getOrder();

    for(auto c : dl.poly.getCoeffs()) {
    	drivingLineMsg.polynom_params.push_back(c);
    }

    drivingLinePub.publish(drivingLineMsg);

#ifdef PUBLISH_DEBUG
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
    bool findIntersectionExit = false;
    std::vector<Segment> foundSegments;
    std::vector<Line*> unusedLines, leftMarkings, midMarkings, rightMarkings, otherMarkings;
    std::vector<cv::RotatedRect> regions;
    std::vector<cv::Point2f> worldPts, imgPts;

//    ROS_INFO("------------- New image ------------------");
    ///////////////////////////////////////////////////////
    // Find lane markings based on distance and angel to each other
    ///////////////////////////////////////////////////////
    std::vector<int> indicesToKeep;

    for(int i = 0; i < lines.size(); i++) {
        float lineAngle = lines.at(i).getAngle();
        for(int j = i + 1; j < lines.size(); j++) {
            float angleDiff = fabsf(lineAngle - lines.at(j).getAngle());
            if(angleDiff < ((3.f / 180.f) * M_PI)) {
                float lineDist = distanceBetweenLines(lines.at(i), lines.at(j));
                if(lineDist < 0.05f && lineDist > 0.02f) {
                    // TODO: maybe make sure the two lines are beside each other, not connecting to a longer line
                    indicesToKeep.push_back(i);
                    indicesToKeep.push_back(j);
                }
            }
        }
    }

    for(auto i : indicesToKeep) {
        unusedLines.push_back(&lines.at(i));
    }
//    for(int i = 0; i < lines.size(); i++) {
//    	unusedLines.push_back(&lines.at(i));
//    }

    // ================================
    // Find lane in new image
    // ================================
    roadModel.getSegmentSearchStart(segStartWorld, segAngle);

    // Draw the segment search start point
    worldPts.push_back(segStartWorld);
    image_operator_.worldToWarpedImg(worldPts, imgPts);
    cv::drawMarker(debugImg_, imgPts.at(0), cv::Scalar(0,255), cv::MARKER_TILTED_CROSS, 8, 3);

    while((totalSegLength < maxSenseRange_) || findIntersectionExit) {
    	// clear the marking vectors. Otherwise the old ones stay in there.
    	leftMarkings.clear();
    	midMarkings.clear();
    	rightMarkings.clear();
    	otherMarkings.clear();

        regions = buildRegions(segStartWorld, segAngle);
		assignLinesToRegions(&regions, unusedLines, leftMarkings, midMarkings, rightMarkings, otherMarkings);
        auto seg = findLaneWithRansac(leftMarkings, midMarkings, rightMarkings, segStartWorld, segAngle);
        // If this is the first segment we can use nullptr as argument for previousSegment
        if(roadModel.segmentFitsToPrevious(
        		foundSegments.empty() ?  nullptr : &foundSegments.back(), // prev segment or nullptr if empty
				&seg,
				foundSegments.empty())) // first segment?
        {
        	foundSegments.push_back(seg);
        	segAngle = seg.angleTotal; // the current angle of the lane
        } else {
        	break;
        }

        totalSegLength += seg.length;
        // Move the segment start forward
        segStartWorld.x = seg.positionWorld.x + cos(seg.angleTotal) * seg.length;
        segStartWorld.y = seg.positionWorld.y + sin(seg.angleTotal) * seg.length;

#if 0
        // ================================
        // Search for an intersection
		// ================================
        Segment intersectionSegment;
        findIntersectionExit = false;
        if(findIntersection(intersectionSegment, segAngle, segStartWorld, leftMarkings, midMarkings, rightMarkings)) {
        	if(roadModel.segmentFitsToPrevious(&foundSegments.back(), &intersectionSegment, false)) {
        		foundSegments.push_back(intersectionSegment);
        		totalSegLength += intersectionSegment.length;
        		segAngle = intersectionSegment.angleTotal; // the current angle of the lane
        		// Move the segment start forward
        		segStartWorld.x = intersectionSegment.positionWorld.x + cos(intersectionSegment.angleTotal) * intersectionSegment.length;
				segStartWorld.y = intersectionSegment.positionWorld.y + sin(intersectionSegment.angleTotal) * intersectionSegment.length;
        		findIntersectionExit = true;
        	} else {
        		ROS_WARN("Intersection segment not plausible");
        		break;
        	}
        }
#endif

        // only used unused lines for the next iteration
//        unusedLines = otherMarkings;
    }

//    ROS_INFO("Detection range = %.2f", totalSegLength);
    bool segmentsValid = roadModel.addSegments(foundSegments, imgTimestamp);

#ifdef PUBLISH_DEBUG
    auto transformedLane = roadModel.getDrivingLinePts();

    worldPts.clear();
    imgPts.clear();

    if(!transformedLane.empty()) {
    	for(int i = 0; i < transformedLane.size(); i++) {
    		worldPts.push_back(cv::Point2f(transformedLane.at(i).x(), transformedLane.at(i).y()));
    	}
    	image_operator_.worldToWarpedImg(worldPts, imgPts);

        cv::Scalar trajColor = segmentsValid ? cv::Scalar(0,255) : cv::Scalar(255);
    	for(int i = 1; i < imgPts.size(); i++) {
            cv::circle(debugImg_, imgPts.at(i), 5, trajColor, 2, cv::LINE_AA);
            cv::line(debugImg_, imgPts.at(i-1), imgPts.at(i), trajColor, 2, cv::LINE_AA);
    	}
    }

//    if(foundSegments.size() > 0) {
//    	// ego lane in orange
//    	cv::circle(debugImg_, foundSegments.at(0).positionImage, 5, cv::Scalar(255,128), 2, cv::LINE_AA);
////    	// left lane marking in purple
////    	cv::circle(debugImg_, foundSegments.at(0).leftPosI, 3, cv::Scalar(153,0,204), 1, cv::LINE_AA);
////    	// right lane marking in purple
////    	cv::circle(debugImg_, foundSegments.at(0).rightPosI, 3, cv::Scalar(153,0,204), 1, cv::LINE_AA);
////    	// mid lane marking in yellow
////    	cv::circle(debugImg_, foundSegments.at(0).midPosI, 3, cv::Scalar(255,255), 1, cv::LINE_AA);
//    }
//
//    for(int i = 1; i < foundSegments.size(); i++) {
//    	// ego lane in orange
//    	cv::circle(debugImg_, foundSegments.at(i).positionImage, 5, cv::Scalar(255,128), 2, cv::LINE_AA);
//    	cv::line(debugImg_, foundSegments.at(i-1).positionImage, foundSegments.at(i).positionImage, cv::Scalar(255,128), 2, cv::LINE_AA);
////    	if(!foundSegments.at(i).isIntersection() && !foundSegments.at(i-1).isIntersection()) {
////    		// left lane marking in purple
////    		cv::line(debugImg_, foundSegments.at(i-1).leftPosI, foundSegments.at(i).leftPosI, cv::Scalar(153,0,204), 2, cv::LINE_AA);
////    		cv::circle(debugImg_, foundSegments.at(i).leftPosI, 3, cv::Scalar(153, 0, 204), 1, cv::LINE_AA);
////    		// right lane marking in purple
////    		cv::line(debugImg_, foundSegments.at(i-1).rightPosI, foundSegments.at(i).rightPosI, cv::Scalar(153,0,204), 2, cv::LINE_AA);
////    		cv::circle(debugImg_, foundSegments.at(i).rightPosI, 3, cv::Scalar(153,0,204), 1, cv::LINE_AA);
////    		// mid lane marking in yellow
////    		cv::line(debugImg_, foundSegments.at(i-1).midPosI, foundSegments.at(i).midPosI, cv::Scalar(255,255), 2, cv::LINE_AA);
////    		cv::circle(debugImg_, foundSegments.at(i).midPosI, 3, cv::Scalar(255,255), 1, cv::LINE_AA);
////    	}
//    }

//    for(auto m : unusedLines) {
//    	cv::line(debugImg_, m->iP1_, m->iP2_, cv::Scalar(0,180), 2, cv::LINE_AA);
//    }
#endif
}

///
/// \brief LineDetection::buildRegions
/// \param position Position of the segment in world coordinates
/// \param angle orientation of the segment in [rad]
/// \return the three regions in image coordinates
///
std::vector<cv::RotatedRect> LineDetection::buildRegions(cv::Point2f positionWorld, float angle) {
	int numRegions = 3;
    std::vector<cv::RotatedRect> regions(numRegions);
    std::vector<cv::Point2f> imgPts(1), worldPts(2);
    cv::Vec2f dirVec(cos(angle), sin(angle)); // in world coordinates
    cv::Vec2f leftVec(laneWidthWorld_ * dirVec[1], laneWidthWorld_ * dirVec[0]); // in world coordinates

    // Convert the position to image coordinates
    worldPts.at(0) = positionWorld;
    image_operator_.worldToWarpedImg(worldPts, imgPts);
    cv::Point2f positionImg = imgPts.at(0);

    // Calculate the region size in world coordinates and then convert to size in image coordinates
    cv::Point2f worldPtOuterRegionSize(positionWorld.x + segmentLength_, positionWorld.y + (1.2f * laneWidthWorld_));
    worldPts.at(0) = worldPtOuterRegionSize;
    cv::Point2f worldPtMiddleRegionSize(positionWorld.x + segmentLength_, positionWorld.y + (0.8f * laneWidthWorld_));
    worldPts.at(1) = worldPtMiddleRegionSize;
    image_operator_.worldToWarpedImg(worldPts, imgPts);
    cv::Size outerRegionSize(imgPts.at(0).x - positionImg.x, positionImg.y - imgPts.at(0).y);
    cv::Size innerRegionSize(imgPts.at(1).x - positionImg.x, positionImg.y - imgPts.at(1).y);

    // Get the region points in world coordinates
    worldPts.resize(numRegions);
    imgPts.resize(numRegions);

    cv::Point2f centerMidRegion(positionWorld.x + (0.5f * segmentLength_ * dirVec[0]) - (0.5f * leftVec[0]),
         	    				positionWorld.y + (0.5f * segmentLength_ * dirVec[1]) + (0.5f * leftVec[1]));

    cv::Point2f centerRightRegion(centerMidRegion.x + 1.2f*leftVec[0],
    							  centerMidRegion.y - 1.2f*leftVec[1]);

    cv::Point2f centerLeftRegion(centerMidRegion.x - 1.2f*leftVec[0],
    							 centerMidRegion.y + 1.2f*leftVec[1]);


    worldPts.at(0) = centerLeftRegion;
    worldPts.at(1) = centerMidRegion;
    worldPts.at(2) = centerRightRegion;
    image_operator_.worldToWarpedImg(worldPts, imgPts);

    // Create the regions
    regions.at(0) = cv::RotatedRect(imgPts.at(0), outerRegionSize, (-angle * 180.f / M_PI));
    regions.at(1) = cv::RotatedRect(imgPts.at(1), innerRegionSize, (-angle * 180.f / M_PI));
    regions.at(2) = cv::RotatedRect(imgPts.at(2), outerRegionSize, (-angle * 180.f / M_PI));

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

    return regions;
}

///
/// \brief LineDetection::assignLinesToRegions assigns all lines to one of the three regions (left, middle, right)
/// or to others.
/// \param regions three regions (left, middle, right)
/// \param lines the lines which should be assigned to the regions
///
void LineDetection::assignLinesToRegions(std::vector<cv::RotatedRect> *regions, std::vector<Line*> &lines,
                                         std::vector<Line*> &leftMarkings, std::vector<Line*> &midMarkings,
										 std::vector<Line*> &rightMarkings, std::vector<Line*> &otherMarkings) {

	float orientationAngle = (regions->at(0).angle / 180.f) * M_PI; // the regions angles in [rad]
	if(orientationAngle > M_PI)
		ROS_WARN("Orientation angle > PI");

    for(auto linesIt = lines.begin(); linesIt != lines.end(); ++linesIt) {
    	// first check if the angle is ok to ignore stop and start lines
    	if(fabsf(orientationAngle - (*linesIt)->getAngle()) > M_PI_2) {
    		otherMarkings.push_back(*linesIt);
    	} else {
    		// if angle is ok, test with regions
    		if(lineIsInRegion(*linesIt, &(regions->at(0)), true)) {
    			leftMarkings.push_back(*linesIt);
    			cv::line(debugImg_, (*linesIt)->iP1_, (*linesIt)->iP2_, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    		} else if(lineIsInRegion(*linesIt, &(regions->at(1)), true)) {
    			midMarkings.push_back(*linesIt);
    			cv::line(debugImg_, (*linesIt)->iP1_, (*linesIt)->iP2_, cv::Scalar(0, 255), 2, cv::LINE_AA);
    		} else if(lineIsInRegion(*linesIt, &(regions->at(2)), true)) {
    			rightMarkings.push_back(*linesIt);
    			cv::line(debugImg_, (*linesIt)->iP1_, (*linesIt)->iP2_, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    		} else {
    			otherMarkings.push_back(*linesIt);
    		}
    	}
    }

    ROS_INFO("---");
    std::vector<cv::Point2f> linePointsWorld;
    for(auto l : leftMarkings) {
    	linePointsWorld.push_back(l->wP1_);
    	linePointsWorld.push_back(l->wP2_);
    }
    if(!linePointsWorld.empty()) {
    	auto rotatedRect = cv::minAreaRect(linePointsWorld);
    	if((rotatedRect.size.height > 0.1f) && (rotatedRect.size.width > 0.1f)) {
    		ROS_INFO("Left markings rect size = (%.3f, %.3f)", rotatedRect.size.width, rotatedRect.size.height);
    		leftMarkings.clear();
    	}
    }

    linePointsWorld.clear();
    for(auto l : midMarkings) {
    	linePointsWorld.push_back(l->wP1_);
    	linePointsWorld.push_back(l->wP2_);
    }
    // TODO: if there is a double line, it could be bigger. test if we need a higher limit here
    if(!linePointsWorld.empty()) {
    	auto rotatedRect = cv::minAreaRect(linePointsWorld);
    	if((rotatedRect.size.height > 0.1f) && (rotatedRect.size.width > 0.1f)) {
    		ROS_INFO("Mid markings rect size = (%.3f, %.3f)", rotatedRect.size.width, rotatedRect.size.height);
    	}
    }

    linePointsWorld.clear();
    for(auto l : rightMarkings) {
    	linePointsWorld.push_back(l->wP1_);
    	linePointsWorld.push_back(l->wP2_);
    }
    if(!linePointsWorld.empty()) {
    	auto rotatedRect = cv::minAreaRect(linePointsWorld);
    	if((rotatedRect.size.height > 0.1f) && (rotatedRect.size.width > 0.1f)) {
    		ROS_INFO("Right markings rect size = (%.3f, %.3f)", rotatedRect.size.width, rotatedRect.size.height);
    		rightMarkings.clear();
    	}
    }
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
//            ROS_INFO_STREAM("Points " << line->iP1_ << " and " << line->iP2_ << " give Rect at " << lineRect.center << " with size " << lineRect.size);
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
                                          cv::Point2f segStartWorld, float prevAngle) {
    float bestAngle = prevAngle;
    float bestScore = 0.f;
    cv::Point2f bestLeft, bestMid, bestRight;
    size_t numLines = leftMarkings.size() + midMarkings.size() + rightMarkings.size();
//    float angleVar = M_PI / 32.0f; // TODO move to config
    size_t maxIterations = 200, iteration = 0;
    std::vector<cv::Point2f> worldPts, imgPts;
    worldPts.push_back(segStartWorld);
    image_operator_.worldToWarpedImg(worldPts, imgPts);

    if(numLines < 5) {
//        ROS_WARN("No lines for Ransac");
        return Segment(segStartWorld, imgPts.at(0), 0.f, prevAngle, segmentLength_, 0.f);
    }

    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0, numLines - 1);

//    ROS_INFO_STREAM("Left markings: " << leftMarkings.size());
//    ROS_INFO_STREAM("Mid markings: " << midMarkings.size());
//    ROS_INFO_STREAM("Right markings: " << rightMarkings.size());
//    if(leftMarkings.size() < 2)
//    	ROS_WARN_STREAM("Only having " << leftMarkings.size() << " left markings");
//    if(midMarkings.size() < 2)
//    	ROS_WARN_STREAM("Only having " << midMarkings.size() << " mid markings");
//    if(rightMarkings.size() < 2)
//    	ROS_WARN_STREAM("Only having " << rightMarkings.size() << " right markings");

    while(bestScore < 0.9 && (iteration++ < maxIterations)) {
        // Select a random line and get its angle
    	// Also get a random point and move it on the expected middle line
        int randomIdx = distribution(generator);
        float currentAngle = prevAngle;
        cv::Vec2f dirVec;
        cv::Point2f randomPoint;

        if(randomIdx < leftMarkings.size()) {
            currentAngle = leftMarkings.at(randomIdx)->getAngle();
            randomPoint =  leftMarkings.at(randomIdx)->wP1_;
            dirVec = cv::Vec2f (cos(currentAngle), sin(currentAngle));
            cv::Vec2f downVec(dirVec[1], - dirVec[0]);
            randomPoint.x += downVec[0] * laneWidthWorld_;
            randomPoint.y += downVec[1] * laneWidthWorld_;
//            ROS_INFO_STREAM("Move left marking from " << leftMarkings.at(randomIdx)->wP1_ << " with " << (downVec * laneWidthWorld_));
        } else if((randomIdx - leftMarkings.size()) < midMarkings.size()){
            currentAngle = midMarkings.at(randomIdx - leftMarkings.size())->getAngle();
            randomPoint =  midMarkings.at(randomIdx - leftMarkings.size())->wP1_;
            dirVec = cv::Vec2f (cos(currentAngle), sin(currentAngle));
            // This point is already on the middle line
//            ROS_INFO_STREAM("Use mid marking " << midMarkings.at(randomIdx - leftMarkings.size())->wP1_);
        } else {
            currentAngle = rightMarkings.at(randomIdx - leftMarkings.size() - midMarkings.size())->getAngle();
            randomPoint =  rightMarkings.at(randomIdx - leftMarkings.size() - midMarkings.size())->wP1_;
            dirVec = cv::Vec2f (cos(currentAngle), sin(currentAngle));
            cv::Vec2f upVec(- dirVec[1], dirVec[0]);
            randomPoint.x += upVec[0] * laneWidthWorld_;
            randomPoint.y += upVec[1] * laneWidthWorld_;
//            ROS_INFO_STREAM("Move right marking from " << rightMarkings.at(randomIdx - leftMarkings.size() - midMarkings.size())->wP1_ <<
//            		" with " << (upVec * laneWidthWorld_));
        }

        // Now build 3 RotatedRects where we expect the lane markings
        cv::Size2f boxSize(segmentLength_ * 2, lineVar_ * 2);
//        ROS_INFO_STREAM("Box size: " << boxSize);
        cv::Vec2f upVec(- dirVec[1], dirVec[0]);
        cv::Point2f leftPos(randomPoint.x + upVec[0] * laneWidthWorld_, randomPoint.y + upVec[1] * laneWidthWorld_);
        cv::Point2f rightPos(randomPoint.x - upVec[0] * laneWidthWorld_, randomPoint.y - upVec[1] * laneWidthWorld_);
        cv::RotatedRect midBox(randomPoint, boxSize, currentAngle * 180.f / M_PI);
        cv::RotatedRect leftBox(leftPos, boxSize, currentAngle * 180.f / M_PI);
        cv::RotatedRect rightBox(rightPos, boxSize, currentAngle * 180.f / M_PI);

//        ROS_INFO_STREAM("center left: " << cv::Point2f(randomPoint.x + dirVec[0] * laneWidthWorld_,
//        											   randomPoint.y + dirVec[1] * laneWidthWorld_));
//        ROS_INFO_STREAM("center mid: " << randomPoint);
//        ROS_INFO_STREAM("center right: " << cv::Point2f(randomPoint.x - dirVec[0] * laneWidthWorld_,
//        											   	randomPoint.y - dirVec[1] * laneWidthWorld_));

#ifdef PUBLISH_DEBUG
//        std::vector<cv::RotatedRect> regions(3);
//        regions.at(0) = midBox;
//        regions.at(1) = leftBox;
//        regions.at(2) = rightBox;
//        for(auto r : regions) {
//        	cv::Point2f edges[4];
//        	r.points(edges);
//
//        	std::vector<cv::Point2f> imgPts, worldPts;
//        	worldPts.push_back(edges[0]);
//        	worldPts.push_back(edges[1]);
//        	worldPts.push_back(edges[2]);
//        	worldPts.push_back(edges[3]);
//        	worldPts.push_back(r.center);
//        	image_operator_.worldToWarpedImg(worldPts, imgPts);
//
//        	for(int i = 0; i < 4; i++)
//        		cv::line(debugImg_, imgPts.at(i), imgPts.at((i+1)%4), cv::Scalar(255,255,255), 2, cv::LINE_AA);
//        	cv::circle(debugImg_, imgPts.at(4), 3, cv::Scalar(255,128), 1, cv::LINE_AA);
//        }
#endif

        // Check for inliers
        int numInliers = 0;
        numInliers += std::count_if(leftMarkings.begin(), leftMarkings.end(),
        		[leftBox, this](Line *l) {return lineIsInRegion(l, &leftBox, false);});
        numInliers += std::count_if(midMarkings.begin(), midMarkings.end(),
        		[midBox, this](Line *l) {return lineIsInRegion(l, &midBox, false);});
        numInliers += std::count_if(rightMarkings.begin(), rightMarkings.end(),
        		[rightBox, this](Line *l) {return lineIsInRegion(l, &rightBox, false);});

        // compare the selected angle to all lines
//        int numInliers = 0;
//        numInliers += std::count_if(leftMarkings.begin(), leftMarkings.end(),
//                                    [currentAngle, angleVar](Line *l) {return std::abs(l->getAngle() - currentAngle) < angleVar;});
//        numInliers += std::count_if(midMarkings.begin(), midMarkings.end(),
//                                    [currentAngle, angleVar](Line *l) {return std::abs(l->getAngle() - currentAngle) < angleVar;});
//        numInliers += std::count_if(rightMarkings.begin(), rightMarkings.end(),
//                                    [currentAngle, angleVar](Line *l) {return std::abs(l->getAngle() - currentAngle) < angleVar;});

        float newScore = static_cast<float>(numInliers) / static_cast<float>(numLines);
        if(newScore > bestScore) {
            bestScore = newScore;
            bestAngle = currentAngle;
            bestLeft = leftPos;
            bestMid = randomPoint;
            bestRight = rightPos;
        }
    }

//    ROS_INFO_STREAM("Best score: " << bestScore << " with " << (bestScore*(float)numLines) << "/" << numLines <<  " lines");

    // Calculate the segments position based on the results
    // First the a point on the right lane
    cv::Point2f laneMidPt = bestRight;
    cv::Vec2f rightToMidVec(bestMid.x - bestRight.x, bestMid.y - bestRight.y);
    laneMidPt.x += 0.5 * rightToMidVec[0];
    laneMidPt.y += 0.5 * rightToMidVec[1];
    // Move this point back so it is at the segments start
//    ROS_INFO_STREAM("Old position: " << pos << " new one: " << laneMidPt);
    cv::Vec2f segDir(cos(bestAngle), sin(bestAngle)); // orientation vector of segment
    cv::Vec2f vecToInitPos(laneMidPt.x - segStartWorld.x, laneMidPt.y - segStartWorld.y); // vector from init position to new position
    float distToInitPos = sqrt(vecToInitPos[0]*vecToInitPos[0] + vecToInitPos[1]*vecToInitPos[1]);
    if(laneMidPt.x < segStartWorld.x) {
    	laneMidPt.x += segDir[0] * distToInitPos;
    	laneMidPt.y += segDir[1] * distToInitPos;
    } else {
    	laneMidPt.x -= segDir[0] * distToInitPos;
    	laneMidPt.y -= segDir[1] * distToInitPos;
    }
//    ROS_INFO_STREAM("Corrected to: " << laneMidPt);

//    auto angleBetweenVecs = acos(segDir.dot(vecToInitPos));
//    ROS_INFO_STREAM("angle between vecs = " << (angleBetweenVecs * 180.f / M_PI));

//    ROS_INFO_STREAM("Move from " << laneMidPt << " with " << (segDir * distToInitPos) << " with goal " << pos);

//    ROS_INFO_STREAM("Ended up at " << laneMidPt);

//    float t = (laneMidPt.x - segStartWorld.x) / segDir[0];
//    laneMidPt.x += t * segDir[0];
//    laneMidPt.y += t * segDir[1];
    // Convert all needed points to image coordinates
    worldPts.clear();
    imgPts.clear();
    worldPts.push_back(laneMidPt);
    worldPts.push_back(bestLeft);
    worldPts.push_back(bestMid);
    worldPts.push_back(bestRight);
    image_operator_.worldToWarpedImg(worldPts, imgPts);

    auto lengthVec = bestMid - segStartWorld;
    auto segLen = sqrt(lengthVec.x*lengthVec.x + lengthVec.y*lengthVec.y);

    // Create the segment
    Segment s(laneMidPt, imgPts.at(0), prevAngle - bestAngle, bestAngle, segLen, bestScore);
    s.leftPosW = bestLeft;
    s.midPosW = bestMid;
    s.rightPosW = bestRight;
    s.leftPosI = imgPts.at(1);
    s.midPosI = imgPts.at(2);
    s.rightPosI = imgPts.at(3);

    return s;
}

bool LineDetection::findIntersection(Segment &resultingSegment, float segmentAngle, cv::Point2f segStartWorld,
		std::vector<Line*> &leftMarkings, std::vector<Line*> &midMarkings, std::vector<Line*> &rightMarkings) {

	bool foundIntersection = false;
	bool middleExists = false, rightExists = false, leftExists = false;
	float stopLineAngle = 0.f;
	float stopLineXpos = 0.f;
	int numStopLines = 0;

	for(auto l : leftMarkings) {
		if(std::abs(l->getAngle() - segmentAngle - M_PI_2) < (M_PI  / 7.0f)) {
			leftExists = true;
			cv::line(debugImg_, l->iP1_, l->iP2_, cv::Scalar(155), 2, cv::LINE_AA);
		}
	}

	for(auto l : midMarkings) {
		if(std::abs(l->getAngle() - segmentAngle - M_PI_2) < (M_PI  / 7.0f)) {
			middleExists = true;
			numStopLines++;
			stopLineAngle += l->getAngle();
			stopLineXpos += l->wP1_.x;
			stopLineXpos += l->wP2_.x;
			cv::line(debugImg_, l->iP1_, l->iP2_, cv::Scalar(57,189,37), 2, cv::LINE_AA);
		}
	}

	for(auto l : rightMarkings) {
		if(std::abs(l->getAngle() - segmentAngle - M_PI_2) < (M_PI  / 7.0f)) {
			rightExists = true;
			cv::line(debugImg_, l->iP1_, l->iP2_, cv::Scalar(0,255,255), 2, cv::LINE_AA);
		}
	}

	// Build a segment for the whole intersection
	Eigen::Vector2f laneDir;
	SegmentType intersectionType;
	float drivingDirectionAngle = segmentAngle;
	if(middleExists && (leftExists || rightExists)) {
//		ROS_INFO("Intersection found - stop");
		foundIntersection = true;
		intersectionType = SegmentType::INTERSECTION_STOP;
		stopLineAngle = stopLineAngle / static_cast<float>(numStopLines);
		drivingDirectionAngle = stopLineAngle - M_PI_2;
//		laneDir = Eigen::Vector2f(sin(stopLineAngle), - cos(stopLineAngle)); // direction of vector perpendicular to stop line
		laneDir = Eigen::Vector2f(cos(drivingDirectionAngle), sin(drivingDirectionAngle));
	}
	if(!middleExists && leftExists && rightExists) {
//		ROS_INFO("Intersection found - do not stop");
		foundIntersection = true;
		intersectionType = SegmentType::INTERSECTION_GO_STRAIGHT;
		laneDir = Eigen::Vector2f(cos(segmentAngle), sin(segmentAngle));
	}

	if(foundIntersection) {
		std::vector<cv::Point2f> imgPts, worldPts;
		float segmentLength = laneWidthWorld_ * 2.5f;

		worldPts.push_back(segStartWorld);
		image_operator_.worldToWarpedImg(worldPts, imgPts);
		// Segment(cv::Point2f worldPos, cv::Point2f imagePos, float angleDiff_, float angleTotal_, float len, float prob)
		resultingSegment = Segment(segStartWorld, imgPts.at(0), segmentAngle - drivingDirectionAngle,
				drivingDirectionAngle, segmentLength, 1.0f);
		resultingSegment.segmentType = intersectionType;

		// TEST FOR POSITION
		cv::Point2f intersectionPos(stopLineXpos * .5, segStartWorld.y);

		worldPts.resize(4);
		imgPts.clear();

		worldPts.at(0) = cv::Point2f(intersectionPos.x, intersectionPos.y - 0.5f*laneWidthWorld_);
		worldPts.at(1) = cv::Point2f(intersectionPos.x, intersectionPos.y + 1.5f*laneWidthWorld_);
		worldPts.at(2) = cv::Point2f(intersectionPos.x + 2.0f*laneWidthWorld_, intersectionPos.y + 1.5f*laneWidthWorld_);
		worldPts.at(3) = cv::Point2f(intersectionPos.x + 2.0f*laneWidthWorld_, intersectionPos.y - 0.5f*laneWidthWorld_);
		image_operator_.worldToWarpedImg(worldPts, imgPts);

		for(int i = 0; i < 4; i++) {
			cv::line(debugImg_, imgPts[i], imgPts[(i+1)%4], cv::Scalar(0,255));
		}
	}

	return foundIntersection;
}

//cv::Point2f LineDetection::findTrajectoryPoint(std::vector<tf::Stamped<tf::Point>> &drivingLine) {
//	if(drivingLine.empty()) {
//		return cv::Point2f(1.f, 0.f);
//	}
//
//	// TEST POLYFIT
//	int order = 4;
//	cv::Mat srcX(drivingLine.size(), 1, CV_32FC1);
//	cv::Mat srcY(drivingLine.size(), 1, CV_32FC1);
//	cv::Mat dst(order+1, 1, CV_32FC1);
//
//	for(int i = 0; i < drivingLine.size(); i++) {
//		srcX.at<float>(i, 0) = drivingLine.at(i).x();
//		srcY.at<float>(i, 0) = drivingLine.at(i).y();
//	}
//
////	ROS_INFO_STREAM("srcX dims = " << srcX.size << "; srcY dims = " << srcY.size << "; dst dims = " << dst.size);
//	polyfit(srcX, srcY, dst, order);
////	ROS_INFO_STREAM("PolyFit = " << dst);
//
//	std::vector<cv::Point2f> imagePoints, worldPoints;
//
//	ROS_INFO("----------------------------");
//	if(prevPolyCoeff.rows == (order+1)) {
//		auto a = prevPolyCoeff.at<float>(0,0);
//		auto b = prevPolyCoeff.at<float>(1,0);
//		auto c = prevPolyCoeff.at<float>(2,0);
//		auto d = prevPolyCoeff.at<float>(3,0);
//		auto e = prevPolyCoeff.at<float>(4,0);
//
//		for(int i = 2; i < 10; i++) {
//			auto x = i * 0.2;
//			worldPoints.push_back(cv::Point2f(x, e*x*x*x*x + d*x*x*x + c*x*x + b*x + a));
//		}
//
//	}
//
//	prevPolyCoeff = dst;
//
//	auto a = dst.at<float>(0,0);
//	auto b = dst.at<float>(1,0);
//	auto c = dst.at<float>(2,0);
//	auto d = dst.at<float>(3,0);
//	auto e = dst.at<float>(4,0);
//
//	for(int i = 2; i < 10; i++) {
//		auto x = i * 0.2;
//		worldPoints.push_back(cv::Point2f(x, e*x*x*x*x + d*x*x*x + c*x*x + b*x + a));
//	}
//
//	// COMPARE THE POLYNOMS
//	float sqrdError = 0.f;
//	for(int i = 0; i < worldPoints.size() / 2; i++) {
//		auto xDiff = worldPoints.at(i).x - worldPoints.at(i + worldPoints.size()/2).x;
//		auto yDiff = worldPoints.at(i).y - worldPoints.at(i + worldPoints.size()/2).y;
//		sqrdError += xDiff*xDiff + yDiff*yDiff;
//	}
//
//	ROS_INFO("Error = %f", sqrt(sqrdError));
//
//	image_operator_.worldToWarpedImg(worldPoints, imagePoints);
//	for(int i = 0; i < imagePoints.size(); i++) {
////	for(auto p : imagePoints) {
//		cv::circle(debugImg_, imagePoints.at(i), 5,
//				(i<8) ? cv::Scalar(0,0,255) : cv::Scalar(0,255),
//				2, cv::LINE_AA);
//	}
//
//
//	float x = trajectoryDist;
//	return cv::Point2f(x, e*x*x*x*x + d*x*x*x + c*x*x + b*x + a);
//}

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

    image_operator_.warpedImgToWorld(imagePoints, worldPoints);

    // Build lines from points
    std::vector<Line> lines;
    for(size_t i = 0; i < worldPoints.size(); i += 2) {
        houghLines.push_back(Line(imagePoints.at(i), imagePoints.at(i + 1), worldPoints.at(i), worldPoints.at(i + 1)));
    }

#if 0
    // Split lines which are longer than segmentLength (world coordinates)
    worldPoints.clear();
    imagePoints.clear();
    for(auto it = lines.begin(); it != lines.end(); ++it) {
    	int splitInto = static_cast<int>(it->getWorldLength() / segmentLength_) + 1;
//    	ROS_INFO_STREAM("Length = " << it->getWorldLength() << " split into " << splitInto);

    	if(splitInto > 1) {
    		// Build normalized vector
    		cv::Vec2f dir = it->wP2_ - it->wP1_;
    		auto length = sqrt(dir[0]*dir[0] + dir[1]*dir[1]);
    		dir[0] = dir[0] / length;
    		dir[1] = dir[1] / length;

    		cv::Point2f curPos = it->wP1_;
    		for(int i = 0; i < (splitInto - 1); i++) {
    			worldPoints.push_back(curPos);
    			curPos.x += dir[0] * segmentLength_;
    			curPos.y += dir[1] * segmentLength_;
    			worldPoints.push_back(curPos);
    		}
    		// This is the last part of the split line
    		worldPoints.push_back(curPos);
    		worldPoints.push_back(it->wP2_);
    	} else {
    		houghLines.push_back(*it);
    	}
    }

    // Build split lines
    image_operator_.worldToWarpedImg(worldPoints, imagePoints);
    for(size_t i = 0; i < worldPoints.size(); i += 2) {
    	houghLines.push_back(Line(imagePoints.at(i), imagePoints.at(i + 1), worldPoints.at(i), worldPoints.at(i + 1)));
    }

#endif
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
    lineVar_ = config.lane_var;
    lineAngle_ = config.lineAngle;
    maxSenseRange_ = config.maxSenseRange;
    cannyThreshold_ = config.cannyThreshold;
    houghThresold_ = config.houghThreshold;
    houghMinLineLen_ = config.houghMinLineLen;
    houghMaxLineGap_ = config.houghMaxLineGap;
    segmentLength_ = config.segmentLength;
    maxRansacInterations_ = config.ransacIterations;
    trajectoryDist = config.trajectory_dist;
    targetVelocity = config.target_velocity;

    roadModel.setLaneWidth(laneWidthWorld_);
}

} // namespace drive_ros_image_recognition
