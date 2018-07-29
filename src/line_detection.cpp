#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <chrono>
#include <pluginlib/class_list_macros.h>
#include "drive_ros_image_recognition/line_detection.h"
#include "drive_ros_image_recognition/geometry_common.h"
#include "drive_ros_msgs/RoadLine.h"

namespace drive_ros_image_recognition {

LineDetection::LineDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh)
    : nh_(nh)
    , pnh_(pnh)
    , isObstaceCourse(false)
    , driveState(DriveState::Street) // TODO: this should be StartBox in real conditions
    , imageTransport_(pnh)
    , stopLineCount(0)
    , lineWidth_(0.4)
    , image_operator_()
    , dsrv_server_()
    , dsrv_cb_(boost::bind(&LineDetection::reconfigureCB, this, _1, _2))
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
    imageSubscriber_ = imageTransport_.subscribe("/img_in", 10, &LineDetection::imageCallback, this);
    ROS_INFO_STREAM("Subscribed image transport to topic " << imageSubscriber_.getTopic());

    line_output_pub_ = nh_.advertise<drive_ros_msgs::RoadLine>("roadLine", 10);
    ROS_INFO_STREAM("Advertising road line on " << line_output_pub_.getTopic());

#ifdef PUBLISH_DEBUG
    debugImgPub_ = imageTransport_.advertise("debug_image", 10);
    ROS_INFO_STREAM("Publishing debug image on topic " << debugImgPub_.getTopic());
#endif

    // common image operations
    if(!image_operator_.init()) {
        ROS_WARN_STREAM("Failed to init image_operator");
        return false;
    }

    return true;
}

///
/// \brief LineDetection::imageCallback
/// Called for incoming camera image. Extracts the image from the incoming message.
/// \param imgIn the incoming camera image message.
///
void LineDetection::imageCallback(const sensor_msgs::ImageConstPtr &imgIn) {
    // TODO: so far these states are only for the free drive (w/o obstacles) and probably not even complete, yet
    auto currentImage = convertImageMessage(imgIn);
    imgHeight_ = currentImage->rows;
    imgWidth_ = currentImage->cols;
    std::vector<Line> linesInImage;
    findLinesWithHough(currentImage, linesInImage);
    if(driveState == DriveState::StartBox) {
        //    findLanesFromStartbox(linesInImage);
    } else if(driveState == DriveState::Street) {
        findLaneAdvanced(linesInImage);
    } else if(driveState == DriveState::Parking) {
        //    findLaneSimple(linesInImage);
    } else if(driveState == DriveState::Intersection) {
        findIntersectionExit(linesInImage);
    } else {

    }

#ifdef PUBLISH_DEBUG
    debugImgPub_.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, debugImg_).toImageMsg());
    //    cv::namedWindow("Debug Image", CV_WINDOW_NORMAL);
    //    cv::imshow("Debug Image", debugImg_);
    //    cv::waitKey(1);
#endif

    //  auto t_start = std::chrono::high_resolution_clock::now();
    //  findLane();
    //  auto t_end = std::chrono::high_resolution_clock::now();
    //  ROS_INFO_STREAM("Cycle time: " << (std::chrono::duration<double, std::milli>(t_end-t_start).count()) << "ms");
}

///
/// \brief LineDetection::findLinesWithHough
/// Extract Lines from the image using Hough Lines
/// \param img A CvImagePtr to the current image where we want to search for lines
/// \param houghLines A std::vector where the Lines will be returned. Has to be empty.
///
void LineDetection::findLinesWithHough(CvImagePtr img, std::vector<Line> &houghLines) {
    cv::Mat processingImg;
#ifdef PUBLISH_DEBUG
    cv::cvtColor(*img, debugImg_, CV_GRAY2RGB);
#endif

    // Blur the image and find edges with Canny
    cv::GaussianBlur(*img, processingImg, cv::Size(15, 15), 0, 0);
    cv::Canny(processingImg, processingImg, cannyThreshold_, cannyThreshold_ * 3, 3);

    // Zero out the vehicle
    int imgWidth = processingImg.cols;
    int imgHeight = processingImg.rows;
    cv::Point vehiclePolygon[1][4];
    // TODO: these factor should be in a config
    vehiclePolygon[0][0] = cv::Point(imgWidth * .35, imgHeight);
    vehiclePolygon[0][1] = cv::Point(imgWidth * .65, imgHeight);
    vehiclePolygon[0][2] = cv::Point(imgWidth * .65, imgHeight * .8);
    vehiclePolygon[0][3] = cv::Point(imgWidth * .35, imgHeight * .8);
    const cv::Point* polygonStarts[1] = { vehiclePolygon[0] };
    int polygonLengths[] = { 4 };
    cv::fillPoly(processingImg, polygonStarts, polygonLengths, 1, cv::Scalar(), cv::LINE_8);

    // Get houghlines
    std::vector<cv::Vec4i> hLinePoints;
    cv::HoughLinesP(processingImg, hLinePoints, 1, CV_PI / 180, houghThresold_, houghMinLineLen_, houghMaxLineGap_);

    // Extract points for houghLines and convert to world-coordinates
    std::vector<cv::Point2f> imagePoints, worldPoints;
    for(size_t i = 0; i < hLinePoints.size(); i++) {
        cv::Vec4i currentPoints = hLinePoints.at(i);
        imagePoints.push_back(cv::Point(currentPoints[0], currentPoints[1]));
        imagePoints.push_back(cv::Point(currentPoints[2], currentPoints[3]));
    }
    image_operator_.imageToWorld(imagePoints, worldPoints);

    // Build lines from points to return them
    for(size_t i = 0; i < worldPoints.size(); i += 2) {
        houghLines.push_back(Line(imagePoints.at(i), imagePoints.at(i + 1), worldPoints.at(i), worldPoints.at(i + 1), Line::LineType::UNKNOWN));
    }

    return;
}

///
/// \brief LineDetection::findLaneAdvanced
/// In this scenario lines can be missing, we could have a double middle line, intersection lines,
/// and the start line could occur.
/// \param lines
/// \return
///
bool LineDetection::findLaneAdvanced(std::vector<Line> &lines) {
    // Last frames middle line is our new guess. Clear old middle line
    std::vector<Line> currentGuess;
    if(!currentMiddleLine_.empty()) {
        for(auto l : currentMiddleLine_) {
            l.lineType_ = Line::LineType::GUESS;
            currentGuess.push_back(l);
        }
        currentMiddleLine_.clear();
    }

    // todo: we should only do this, when the car starts in the start box. Otherwise we can get in trouble
    std::vector<cv::Point2f> imagePoints, worldPoints;
    if(currentGuess.empty()) {
        // todo: we should have a state like "RECOVER", where we try to find the middle line again (more checks, if it is really the middle one)
        worldPoints.push_back(cv::Point2f(0.24, 0.5));
        worldPoints.push_back(cv::Point2f(0.4, 0.5));
        worldPoints.push_back(cv::Point2f(0.55, 0.5));
        worldPoints.push_back(cv::Point2f(0.70, 0.5));
        image_operator_.worldToImage(worldPoints, imagePoints);
        for(size_t i = 1; i < imagePoints.size(); i++)
            currentGuess.push_back(Line(imagePoints.at(i - 1), imagePoints.at(i), worldPoints.at(i - 1), worldPoints.at(i), Line::LineType::GUESS));
    }

    publishDebugLines(currentGuess);

    // Use the guess to classify the lines
    determineLineTypes(lines, currentGuess);
    std::vector<Line> leftLines, middleLines, rightLines, horizontalLines;
    for(auto l : lines) {
        if(l.lineType_ == Line::LineType::MIDDLE_LINE)
            middleLines.push_back(l);
        else if(l.lineType_ == Line::LineType::LEFT_LINE)
            leftLines.push_back(l);
        else if(l.lineType_ == Line::LineType::RIGHT_LINE)
            rightLines.push_back(l);
        else if(l.lineType_ == Line::LineType::HORIZONTAL_LEFT_LANE ||
                l.lineType_ == Line::LineType::HORIZONTAL_OUTER_LEFT ||
                l.lineType_ == Line::LineType::HORIZONTAL_RIGHT_LANE ||
                l.lineType_ == Line::LineType::HORIZONTAL_OUTER_RIGHT)
            horizontalLines.push_back(l);
    }

    // Classify the horizontal lines
    // Find start line
    // TODO: check if we have lines on left and right lane. in this case, set all left and right ones (in a certain x-interval to start line)
    for(auto firstIt = horizontalLines.begin(); firstIt != horizontalLines.end(); ++firstIt) {
        for(auto secondIt = horizontalLines.begin(); secondIt != horizontalLines.end(); ++secondIt) {
            if(((firstIt->lineType_ == Line::LineType::HORIZONTAL_LEFT_LANE) && (secondIt->lineType_ == Line::LineType::HORIZONTAL_RIGHT_LANE)) ||
                    ((firstIt->lineType_ == Line::LineType::HORIZONTAL_RIGHT_LANE) && (secondIt->lineType_ == Line::LineType::HORIZONTAL_LEFT_LANE))) {
                if(abs(firstIt->wMid_.x - secondIt->wMid_.x) < 0.2) {
                    firstIt->lineType_ = Line::LineType::START_LINE;
                    secondIt->lineType_ = Line::LineType::START_LINE;
                }
            }
        }
    }

    // Find stop line
    std::vector<float> distsToStopLine;
    for(auto firstIt = horizontalLines.begin(); firstIt != horizontalLines.end(); ++firstIt) {
        if(firstIt->lineType_ == Line::LineType::HORIZONTAL_RIGHT_LANE) {
            bool foundRightOuterBound = false, foundLeftOuterBound = false;
            for(auto secondIt = horizontalLines.begin(); secondIt != horizontalLines.end(); ++secondIt) {
                if(secondIt->lineType_ == Line::LineType::HORIZONTAL_OUTER_LEFT)
                    foundLeftOuterBound = true;
                else if(secondIt->lineType_ == Line::LineType::HORIZONTAL_OUTER_RIGHT)
                    foundRightOuterBound = true;
            }
            if(foundLeftOuterBound && foundRightOuterBound) {
                firstIt->lineType_ = Line::LineType::STOP_LINE;
                distsToStopLine.push_back(firstIt->wMid_.x);
            }
        }
    }
    if(distsToStopLine.size() > 2) {
        auto avgDistToStopLine = std::accumulate(distsToStopLine.begin(), distsToStopLine.end(), 0.0f) / distsToStopLine.size();
        ROS_INFO_STREAM("Found a stop line; distance to it = " << avgDistToStopLine);
        stopLineCount++;
        if(avgDistToStopLine <= 0.6f) { // TODO: magic number

            if(stopLineCount > 5) {
                driveState = DriveState::Intersection;
                // todo: create intersection exit based on stop line (orthogonal) or middle line?
            }
            // todo: maybe return here? but do not forget to publish road points
        }
    } else {
        stopLineCount = 0;
    }

#ifdef PUBLISH_DEBUG
    publishDebugLines(rightLines);
    publishDebugLines(leftLines);
    publishDebugLines(horizontalLines);
#endif

    if(middleLines.empty()/* && leftLines.empty() && rightLines.empty()*/) {
        // TODO: this should not happen. maybe set state to sth. like RECOVER
        return false;
    }

    // ------------------------------------------------------------------------------------------------------------------------------------
    // TODO: find a better and more efficient way to do this. maybe also project left and right line into middle to use all points.
    // ------------------------------------------------------------------------------------------------------------------------------------
    // in the free drive event, double lines are treated as normal dashed lines
    // sort our middle lines based on wP1_.x of line
    std::sort(middleLines.begin(), middleLines.end(), [](Line &a, Line &b){ return a.wP1_.x < b.wP1_.x; });
    // build bounding boxes around group of lines, where lines in group are closer than 0.25m to each other
    float currentMinX = middleLines.at(0).wP1_.x;
    std::vector<cv::Point2f> cPts, newMidLinePts;
    for(auto l : middleLines) {
        if(l.wP1_.x > (currentMinX + 0.25)) {
            buildBbAroundLines(cPts, newMidLinePts);
            cPts.clear();
            currentMinX = l.wP1_.x;
        }
        cPts.push_back(l.wP1_);
        cPts.push_back(l.wP2_);
    }

    // finish the bounding box building step
    if(!cPts.empty()) {
        buildBbAroundLines(cPts, newMidLinePts);
    }

    // build the middle line from out points
    // extend the last line to the front
    auto wPt1 = newMidLinePts.at(newMidLinePts.size() - 2);
    auto wPt2 = newMidLinePts.at(newMidLinePts.size() - 1);
    auto dir = wPt2 - wPt1;
    auto dirLen = sqrt(dir.x * dir.x + dir.y * dir.y);

    auto newPt = wPt2 + (dir * (0.6 / dirLen));
    if(newPt.x < maxViewRange_)
        newMidLinePts.push_back(newPt);

    // extend first line to the back
    wPt1 = newMidLinePts.at(0);
    wPt2 = newMidLinePts.at(1);
    dir = wPt2 - wPt1;
    dirLen = sqrt(dir.x * dir.x + dir.y * dir.y);
    // the image starts 0.24m from the rear_axis_middle_ground. TODO: sould be in config
    auto distToImageBoundary = wPt1.x - 0.24;
    if(distToImageBoundary > 0.1) { // distance is positive
        // dir, distToImageBoundary, dirLen are all positive
        newPt = wPt1 - (dir * distToImageBoundary / dirLen);
        newMidLinePts.push_back(newPt);
    }

    // sort the world points based on the distance to the car
    std::sort(newMidLinePts.begin(), newMidLinePts.end(), [](cv::Point2f &a, cv::Point2f &b) { return a.x < b.x; });

    // convert points
    imagePoints.clear();
    image_operator_.worldToImage(newMidLinePts, imagePoints);
    // create the new middle line
    currentMiddleLine_.clear();
    for(size_t i = 1; i < imagePoints.size(); i++) {
        auto a = imagePoints.at(i - 1);
        auto b = imagePoints.at(i);
        // this makes the line coordinates inconsistant, but this should not be a serious problem
        if(a.y > imgHeight_)
            a.y = imgHeight_ - 1;
        if(b.y > imgHeight_)
            b.y = imgHeight_ - 1;

        currentMiddleLine_.push_back(Line(a, b, newMidLinePts.at(i - 1), newMidLinePts.at(i), Line::LineType::MIDDLE_LINE));
    }

    // validate our new guess
    if(!currentMiddleLine_.empty()) {
        if(currentMiddleLine_.at(currentMiddleLine_.size() - 1).wP2_.x < 0.5) {
            // our guess is too short, it is better to use the default one; TODO: bad idea -> RECOVER state
            currentMiddleLine_.clear();
        } else if((currentMiddleLine_.at(0).getAngle() < 1.0) || (currentMiddleLine_.at(0).getAngle() > 2.6)) {
            // the angle of the first segment is weird (TODO: workaround for now, why can this happen?)
            // if we have more than one segment, then we just delete the first
            if(currentMiddleLine_.size() > 1) {
                currentMiddleLine_.erase(currentMiddleLine_.begin());
            } else {
                // otherwise throw the guess away
                currentMiddleLine_.clear();
            }
        }

        bool done = false;
        for(size_t i = 1; i < currentMiddleLine_.size() && !done; i++) {
            // angles between two connected segments should be plausible
            if(std::abs(currentMiddleLine_.at(i - 1).getAngle() - currentMiddleLine_.at(i).getAngle()) > 0.8) {
                // 0.8 = 45 degree
                // todo: clear whole line or just to this segment?
                currentMiddleLine_.erase(currentMiddleLine_.begin() + i, currentMiddleLine_.end());
                //        ROS_INFO_STREAM("clipped line because of angle difference");
                done = true;
            } else if(currentMiddleLine_.at(i).wP2_.x > maxViewRange_) {
                //        ROS_INFO_STREAM("clipped line because of maxViewRange_");
                currentMiddleLine_.erase(currentMiddleLine_.begin() + i, currentMiddleLine_.end());
                done = true;
            }
            else if((i < (currentMiddleLine_.size() - 1)) && currentMiddleLine_.at(i).getLength() > 0.3) {
                // we build middle line segments of max 0.25m, so this is a test; TODO: overthink this
                // excluding the last segment, since we created this for searching forward
                //        ROS_INFO_STREAM("clipped line because of length");
                currentMiddleLine_.erase(currentMiddleLine_.begin() + i, currentMiddleLine_.end());
                done = true;
            }
        }
    }

    publishDebugLines(currentMiddleLine_);

    // publish points to road topic
    // TODO: we want to publish not only the middle line, but also start line, stop lines, intersections, barred areas, ...
    // todo: we should do this outside this function inside the imageCallback
    drive_ros_msgs::RoadLine msgMidLine;
    msgMidLine.lineType = drive_ros_msgs::RoadLine::MIDDLE;
    for(auto l : currentMiddleLine_) {
        geometry_msgs::PointStamped pt1, pt2;
        pt1.point.x = l.wP1_.x;
        pt1.point.y = l.wP1_.y;
        msgMidLine.points.push_back(pt1);
        pt2.point.x = l.wP2_.x;
        pt2.point.y = l.wP2_.y;
        msgMidLine.points.push_back(pt2);
    }

    line_output_pub_.publish(msgMidLine);
}

bool LineDetection::findIntersectionExit(std::vector<Line> &lines) {
    std::vector<Line> potMidLines;

    // we keep current current middle line
    // todo: we should check if the middle line is plausible
    if(currentMiddleLine_.size() == 0) {
        ROS_WARN("No middle line");
        return false;
    }
    for(auto l : lines) {
        if(std::abs(l.getAngle() - currentMiddleLine_.at(0).getAngle()) < (M_PI / 8)) {
            l.lineType_ = Line::LineType::UNKNOWN;
            potMidLines.push_back(l);
        }
    }

    determineLineTypes(potMidLines, currentMiddleLine_);
    float closestLineDist = 10.0f;
    Line *closestMiddleLine;
    bool foundLeft = false, foundMiddle = false, foundRight = false;
    for(auto l : potMidLines) {
        foundLeft |= l.lineType_ == Line::LineType::LEFT_LINE;
        foundMiddle |= l.lineType_ == Line::LineType::MIDDLE_LINE;
        foundRight |= l.lineType_ == Line::LineType::RIGHT_LINE;

        if(l.wMid_.x < closestLineDist) {
            closestLineDist = l.wMid_.x;
            closestMiddleLine = &l;
        }
    }

    ROS_INFO_STREAM("Closest line dist = " << closestLineDist);
    ROS_INFO_STREAM("found left: " << foundLeft << "\tfound middle: " << foundMiddle << "\tfound right: " << foundRight);

    if(foundLeft && foundMiddle && foundRight) {
        closestMiddleLine->lineType_ = Line::LineType::GUESS;
        potMidLines.push_back(*closestMiddleLine);
        driveState = DriveState::Street;
    }

    publishDebugLines(currentMiddleLine_);
    publishDebugLines(potMidLines);

    // todo: ideas: 1) detect opposite stop line as end of intersection
    //              2) usu odometry and knowledge about intersection size (line_width * 2)
    return true;
}

void LineDetection::determineLineTypes(std::vector<Line> &lines, std::vector<Line> &currentGuess) {
    for(auto lineIt = lines.begin(); lineIt != lines.end(); ++lineIt) {
        Line segment = currentGuess.at(0);
        bool isInSegment = false;
        // find the corresponding segment
        // todo: this only uses the x-coordinate right now, improve this
        for(size_t i = 0; (i < currentGuess.size()) && !isInSegment; i++) {
            segment = currentGuess.at(i);
            if(lineIt->wMid_.x > segment.wP1_.x) {
                if(lineIt->wMid_.x < segment.wP2_.x) {
                    isInSegment = true;
                }
            } else if(lineIt->wMid_.x > segment.wP2_.x) {
                isInSegment = true;
            }
        }

        if(isInSegment) {
            // classify the line
            if(std::abs(lineIt->getAngle() - segment.getAngle()) < lineAngle_) {
                // lane width is 0.35 to 0.45 [m]
                // todo: distance should be calculated based on orthogonal distance.
                // the current approach leads to problems in curves
                auto absDistanceToMidLine = std::abs(lineIt->wMid_.y - segment.wMid_.y);
                if(absDistanceToMidLine < (lineWidth_ / 2)) {
                    lineIt->lineType_ = Line::LineType::MIDDLE_LINE;
                } else if((lineIt->wMid_.y < segment.wMid_.y) && (std::abs(absDistanceToMidLine - lineWidth_) < lineVar_)) {
                    lineIt->lineType_ = Line::LineType::RIGHT_LINE;
                } else if((lineIt->wMid_.y > segment.wMid_.y) && (std::abs(absDistanceToMidLine - lineWidth_) < lineVar_)) {
                    lineIt->lineType_ = Line::LineType::LEFT_LINE;
                }
            } else {
                classifyHorizontalLine(&*lineIt, segment.wMid_.y - lineIt->wMid_.y); // convert iterator to pointer
            }
        }
    }
}

void LineDetection::classifyHorizontalLine(Line *line, float worldDistToMiddleLine) {
    if(worldDistToMiddleLine < -lineWidth_) {
        line->lineType_ = Line::LineType::HORIZONTAL_OUTER_LEFT;
    } else if(worldDistToMiddleLine < 0) {
        line->lineType_ = Line::LineType::HORIZONTAL_LEFT_LANE;
    } else if(worldDistToMiddleLine < lineWidth_) {
        line->lineType_ = Line::LineType::HORIZONTAL_RIGHT_LANE;
    } else {
        line->lineType_ = Line::LineType::HORIZONTAL_OUTER_RIGHT;
    }
}

void LineDetection::buildBbAroundLines(std::vector<cv::Point2f> &centerPoints, std::vector<cv::Point2f> &midLinePoints) {
    auto rect = cv::minAreaRect(centerPoints);
    cv::Point2f a, b;
    cv::Point2f vertices2f[4];
    rect.points(vertices2f);

    // use the long side as line
    if(std::abs(vertices2f[0].x - vertices2f[1].x) > std::abs(vertices2f[1].x - vertices2f[2].x)) {
        a = vertices2f[0];
        b = vertices2f[1];
    } else {
        a = vertices2f[1];
        b = vertices2f[2];
    }
    // ensures the correct order of the points in newMidLinePts vector
    if(a.x > b.x) {
        midLinePoints.push_back(b);
        midLinePoints.push_back(a);
    } else {
        midLinePoints.push_back(a);
        midLinePoints.push_back(b);
    }
}

void LineDetection::publishDebugLines(std::vector<Line> &lines) {
    for(auto it = lines.begin(); it != lines.end(); ++it) {
        if(it->lineType_ == Line::LineType::MIDDLE_LINE)
            cv::line(debugImg_, it->iP1_, it->iP2_, cv::Scalar(0, 255), 2, cv::LINE_AA); // Green
        else if((it->lineType_ == Line::LineType::LEFT_LINE) || (it->lineType_ == Line::LineType::RIGHT_LINE))
            cv::line(debugImg_, it->iP1_, it->iP2_, cv::Scalar(0, 0, 255), 2, cv::LINE_AA); // Blue
        else if(it->lineType_ == Line::LineType::STOP_LINE)
            cv::line(debugImg_, it->iP1_, it->iP2_, cv::Scalar(255), 2, cv::LINE_AA); // Red
        else if(it->lineType_ == Line::LineType::START_LINE)
            cv::line(debugImg_, it->iP1_, it->iP2_, cv::Scalar(0, 204, 255), 2, cv::LINE_AA); // Cyan
        else if(it->lineType_ == Line::LineType::HORIZONTAL_OUTER_LEFT)
            cv::line(debugImg_, it->iP1_, it->iP2_, cv::Scalar(255, 255), 2, cv::LINE_AA); // Yellow
        else if(it->lineType_ == Line::LineType::HORIZONTAL_OUTER_RIGHT)
            cv::line(debugImg_, it->iP1_, it->iP2_, cv::Scalar(255, 153, 51), 2, cv::LINE_AA); // Orange
        else if(it->lineType_ == Line::LineType::GUESS)
            cv::line(debugImg_, it->iP1_, it->iP2_, cv::Scalar(153, 0, 204), 2, cv::LINE_AA); // Purple
        else
            cv::line(debugImg_, it->iP1_, it->iP2_, cv::Scalar(255, 51, 204), 2, cv::LINE_AA); // Pink
    }
}

///
/// \brief LineDetection::reconfigureCB
/// Used by the dynamic reconfigure server
/// \param config
/// \param level
///
void LineDetection::reconfigureCB(drive_ros_image_recognition::LineDetectionConfig& config, uint32_t level){
    image_operator_.setConfig(config);
    lineWidth_ = config.lineWidth;
    lineAngle_ = config.lineAngle;
    lineVar_ = config.lineVar;
    maxViewRange_ = config.maxSenseRange;
    cannyThreshold_ = config.cannyThreshold;
    houghThresold_ = config.houghThreshold;
    houghMinLineLen_ = config.houghMinLineLen;
    houghMaxLineGap_ = config.houghMaxLineGap;
}

} // namespace drive_ros_image_recognition
