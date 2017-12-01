#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include "drive_ros_image_recognition/street_crossing.h"
#include <drive_ros_image_recognition/geometry_common.h>
#include <pluginlib/class_list_macros.h>

namespace drive_ros_image_recognition {

StreetCrossingDetection::StreetCrossingDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh)
  : nh_(nh)
  , pnh_(pnh)
  , imageTransport_(pnh)
  , image_operator_()
  , dsrv_server_()
  , config_()
  , road_hints_buffer_()
  , img_sub_()
  , road_sub_()
  , sync_()
  #if defined (DRAW_DEBUG) || defined (PUBLISH_DEBUG)
  , debugImage(0, 0, CV_8UC1)
  #endif
{
}

StreetCrossingDetection::~StreetCrossingDetection() {
}

bool StreetCrossingDetection::init() {
  dsrv_server_.setCallback(boost::bind(&StreetCrossingDetection::reconfigureCB, this, _1, _2));

  image_operator_ = ImageOperator();
  image_operator_.init();

#ifdef PUBLISH_DEBUG
  debugImagePublisher = imageTransport_.advertise("/street_crossing/debug_image", 10);
#endif

  img_sub_.reset(new image_transport::SubscriberFilter(imageTransport_,"/warped_image", 5));
  road_sub_.reset(new message_filters::Subscriber<drive_ros_msgs::RoadLane>(pnh_,"/road_detection/road_in", 5));
  sync_.reset(new message_filters::Synchronizer<SyncImageToHints>(SyncImageToHints(5), *img_sub_, *road_sub_));
  sync_->registerCallback(boost::bind(&StreetCrossingDetection::syncCallback, this, _1, _2));
  return true;
}

void StreetCrossingDetection::syncCallback(const sensor_msgs::ImageConstPtr& img_in, const drive_ros_msgs::RoadLaneConstPtr& road_in) {
  currentImage_ = convertImageMessage(img_in);
  road_hints_buffer_ = road_in->points;
  find();
}

bool StreetCrossingDetection::find() {
  //  std::vector<cv::Point2f> linePoints;
  //  SearchLine mySl;

  //  // use search-line to find stop-line
  //  mySl.iStart = cv::Point2f(currentImage_->cols / 2, currentImage_->rows * .4);
  //  mySl.iEnd = cv::Point2f(currentImage_->cols / 2, currentImage_->rows * .8);

  //  // find line using unified header
  //  search_direction search_dir = search_direction::y;
  //  search_method search_meth = search_method::sobel;
  //  std::vector<cv::Point> image_points;
  //  std::vector<int> line_widths;
  //  image_operator_.findByLineSearch(mySl,*currentImage_,search_dir,search_meth,image_points,line_widths);
  //#if defined(DRAW_DEBUG) || defined(PUBLISH_DEBUG)
  //  cv::Mat debug_image;
  //  cv::cvtColor(*currentImage_, debug_image, CV_GRAY2RGB);
  //  cv::line(debug_image, mySl.iStart, mySl.iEnd, cv::Scalar(255,255,255));
  //  for (auto point: image_points) {
  //    cv::circle(debug_image,point,2,cv::Scalar(0,255,0),2);
  //  }
  //#endif
  //#ifdef DRAW_DEBUG
  //  cv::namedWindow("Crossing Search debug",CV_WINDOW_NORMAL);
  //  cv::imshow("Crossing Search debug",debug_image);
  //  cv::waitKey(1);
  //#endif
  //#ifdef PUBLISH_DEBUG
  //  debugImagePublisher.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_8UC1, debugImage).toImageMsg());
  //#endif

  // test array of lines
  //  image_operator_.debugPointsImage(*currentImage, search_dir, search_meth);

  if (road_hints_buffer_.size() < 4) {
    ROS_ERROR("[crossing detection] Less than 4 points in buffer, skipping search.");
    return false;
  }

  linestring mid;
  std::string frame_id = road_hints_buffer_.front().header.frame_id;
  geometrypointsToLinestring(road_hints_buffer_, mid);

  //trying to find the stop-line
  std::vector<cv::Point3d> mid_camera_frame;
  image_operator_.linestringToImageFrame(mid, mid_camera_frame, frame_id);

  SearchLine mid_line;
  std::vector<cv::Point> image_points;
  std::vector<int> line_widths;

  for(std::size_t i = 3; i < mid_camera_frame.size(); i++) {

    if (!image_operator_.worldToImage(mid_camera_frame[i-1], mid_line.iStart)) {
      ROS_WARN_STREAM("[crossing detection] Unable to transform middle search segment start point "<<mid_camera_frame[i-1]<<" to image frame, skipping segment");
      continue;
    }

    if (!image_operator_.worldToImage(mid_camera_frame[i], mid_line.iEnd)) {
      ROS_WARN_STREAM("[crossing detection] Unable to transform middle search segment end point "<<mid_camera_frame[i]<<" to image frame, skipping segment");
      continue;
    }

    const float iDist = cv::norm(mid_line.iEnd-mid_line.iStart);
    const float wDist = std::sqrt(std::pow(mid[i].x()-mid[i-1].x(),2)+std::pow(mid[i].y()-mid[i-1].y(),2));

    image_operator_.findByLineSearch(mid_line, *currentImage_, search_direction::y, search_method::sobel, image_points, line_widths);

    if (image_points.size() == 1) {
      ROS_DEBUG_STREAM("[street_crossing] Found middle lane at image point: "<<image_points[0]);
      //trying to detect the stopline
      cv::Point middlePosition = image_points[0];
      point_xy middle_world_bottom;
      if (!image_operator_.imageToWorld(middlePosition, middle_world_bottom)) {
        ROS_WARN_STREAM("[street_crossing] Unable to transform middle image point "<<middlePosition<<" back to world");
        continue;
      }

      //now we go one step to the left/right and check if we still can find a point
      // move offsetSide to the sides of the found point on the line, and search with lines offsetAlong length
      linestring image_line, left_line, right_line;
      boost::geometry::append(image_line, mid[i-1]);
      boost::geometry::append(image_line, mid[i]);
      // todo: check if using this offset helps in any way
//      double image_offset = mid[i].y() - middle_world_bottom;
      drive_ros_geometry_common::moveOrthogonal(mid, left_line, config_.offsetSide);
      drive_ros_geometry_common::moveOrthogonal(mid, right_line, -config_.offsetSide);
      drive_ros_geometry_common::rescaleLength(left_line, config_.offsetAlong);
      drive_ros_geometry_common::rescaleLength(right_line, config_.offsetAlong);

      std::vector<cv::Point> points_right, points_left;
      image_operator_.linestringToImageCoordinates(left_line, points_left, frame_id);
      image_operator_.linestringToImageCoordinates(right_line, points_right, frame_id);

      SearchLine left_seachline, right_serachline;
      left_seachline.iStart = points_left[0];
      left_seachline.iEnd = points_left[1];
      right_serachline.iStart = points_right[0];
      right_serachline.iEnd = points_right[1];

      std::vector<cv::Point> detected_points_left, detected_points_right;
      image_operator_.findByLineSearch(left_seachline, *currentImage_, search_direction::y, search_method::sobel, detected_points_left, line_widths);
      image_operator_.findByLineSearch(right_serachline, *currentImage_, search_direction::y, search_method::sobel, detected_points_right, line_widths);

      if (points_right.size() != 1 && points_left.size() != 1){
        //no crossing, something else
        ROS_WARN_STREAM("[street_crossing] Could not confirm crossing, found: "<<points_left.size()<<" left points and "<<points_right.size()<<" right points");
        continue;
      }
      //check if it a crossing, not a startline
      ROS_INFO_STREAM("[street_crossing] Searching for start line");
      // 0.4 offset
      linestring start_linestring;
      drive_ros_geometry_common::moveOrthogonal(mid, start_linestring, config_.crossingCheckOffset);
      drive_ros_geometry_common::rescaleLength(start_linestring, config_.offsetAlong);
      std::vector<cv::Point> points_start;
      image_operator_.linestringToImageCoordinates(start_linestring, points_start, frame_id);
      SearchLine start_search_line;
      start_search_line.iStart = points_start[0];
      start_search_line.iEnd = points_start[1];

      image_operator_.findByLineSearch(start_search_line, *currentImage_, search_direction::y, search_method::sobel, image_points, line_widths);
      if (image_points.size() > 0) {
        ROS_INFO_STREAM("FOUND STARTING LINE!");
        //we found a start line
        //              street_environment::StartLinePtr startline(new street_environment::StartLine());
        //              startline->addSensor("CAMERA");
        //              startline->addPoint(middlePosition);
        //              startline->viewDirection(viewDirection);
        //              startline->width(0.2);
        //              startline->setTrust(1);
        //              env->objects.push_back(startline);
        //              logger.debug("found startline");
        break;
      } else if (image_points.size() > 1) {
        ROS_WARN_STREAM("[street_crossing] Invalid startline, to many points on the left");
        continue;
      }

      //trying to find oposite stop-line
      //check if it a crossing, not a startline
      //starting at the middle of the left lane
      //          const float oppositeLineStart=config().get<float>("oppositeLineStart",0.7);
      //          const float oppositeLineEnd=config().get<float>("oppositeLineEnd",0.9);
      linestring endSearchLine;
      drive_ros_geometry_common::moveVertical(start_linestring, endSearchLine, config_.oppositeLineOffset);
      std::vector<cv::Point> points_opposite;
      image_operator_.linestringToImageCoordinates(start_linestring, points_opposite, frame_id);
      SearchLine opposite_search_line;
      opposite_search_line.iStart = points_opposite[0];
      opposite_search_line.iEnd = points_opposite[1];
      //          getXYfromPoint(middlePosition-orth*0.4+viewDirection*oppositeLineStart,middlePosition-orth*0.4+viewDirection*oppositeLineEnd,xv,yv,homo);
      image_operator_.findByLineSearch(opposite_search_line, *currentImage_, search_direction::y, search_method::sobel, image_points, line_widths);
      if (image_points.size() == 1) {
        ROS_INFO_STREAM("FOUND CROSSING LINE!");
        //              street_environment::CrossingPtr crossing(new street_environment::Crossing());
        //              crossing->addPoint(middlePosition);
        //              crossing->viewDirection(viewDirection);
        //              crossing->width(0.2);
        //              crossing->setTrust(1);
        //              crossing->addSensor("CAMERA");
        //              env->objects.push_back(crossing);
        //              logger.debug("found crossing");
        break;
      } else {
        ROS_WARN_STREAM("[street_crossing] Could not find opposite line!");
      }
    }
  }
  return true;
}

void StreetCrossingDetection::reconfigureCB(drive_ros_image_recognition::CrossingDetectionConfig& config, uint32_t level){
  config_ = config;
}

void StreetCrossingDetectionNodelet::onInit() {
  street_crossing_detection_.reset(new StreetCrossingDetection(getNodeHandle(),getPrivateNodeHandle()));
  if (!street_crossing_detection_->init()) {
    ROS_ERROR("StreetCrossing nodelet failed to initialize");
    // nodelet failing will kill the entire loader anyway
    ros::shutdown();
  }
  else {
    ROS_INFO("StreetCrossing detection nodelet succesfully initialized");
  }
}

} // namespace drive_ros_image_recognition

PLUGINLIB_EXPORT_CLASS(drive_ros_image_recognition::StreetCrossingDetectionNodelet, nodelet::Nodelet)
