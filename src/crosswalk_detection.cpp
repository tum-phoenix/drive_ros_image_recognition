#include <drive_ros_image_recognition/crosswalk_detection.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pluginlib/class_list_macros.h>

namespace drive_ros_image_recognition {

CrosswalkDetection::CrosswalkDetection() :
  Detection()
{
}

CrosswalkDetection::CrosswalkDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh, image_transport::ImageTransport *it):
  Detection(nh, pnh, it)
{
}

CrosswalkDetection::~CrosswalkDetection() {
}

// debug callback, creates a dummy line
void CrosswalkDetection::imageCallback(const sensor_msgs::ImageConstPtr& img_in) {
  current_image_ = convertImageMessage(img_in);

  // fill hints with dummy points
  road_hints_buffer_.clear();
  geometry_msgs::PointStamped temp_point;
  temp_point.header.frame_id = image_operator_.getWorldFrame();
  temp_point.point.x = 0.1;
  temp_point.point.y = 0;
  temp_point.point.z = 0;
  road_hints_buffer_.push_back(temp_point);
  road_hints_buffer_.push_back(temp_point);
  road_hints_buffer_.push_back(temp_point);
  temp_point.point.x = 0.3;
  road_hints_buffer_.push_back(temp_point);

  // crop image by 64 from all directions and find
  int rect_offset = 64;
  image_operator_.setImageRect(cv::Rect(0, rect_offset, current_image_->cols, current_image_->rows-rect_offset));
  find();
  return;
}

bool CrosswalkDetection::find() {
  if (road_hints_buffer_.size() < 2) {
    ROS_ERROR("[crosswalk detection] Less than 2 points in buffer, skipping search.");
    return false;
  }

  linestring mid, left, right;
  std::string frame_id = road_hints_buffer_.front().header.frame_id;
  geometrypointsToLinestring(road_hints_buffer_, mid);
  drive_ros_geometry_common::moveOrthogonal(mid, left, config_.searchLineWidth/2);
  drive_ros_geometry_common::moveOrthogonal(mid, right, -config_.searchLineWidth/2);

  //trying to find the stop-line
  std::vector<cv::Point3d> search_points_left, search_points_right;
  image_operator_.linestringToImageFrame(left, search_points_left, frame_id);
  image_operator_.linestringToImageFrame(right, search_points_right, frame_id);

  if (!search_points_left.size() == search_points_right.size()) {
    ROS_WARN("[crosswalk detection] Left and right lane image buffer sizes do not match!");
    return false;
  }

  SearchLine search_line;
  std::vector<cv::Point> image_points;
  std::vector<int> line_widths;

  // draw found points
#if defined(DRAW_DEBUG) || defined(PUBLISH_DEBUG)
  cv::namedWindow("Crosswalk detection", CV_WINDOW_NORMAL);
  cv::Mat crosswalk_points_mat = current_image_->clone();
  cv::cvtColor(crosswalk_points_mat, crosswalk_points_mat, cv::COLOR_GRAY2BGR);
#endif

  for(std::size_t i = 3; i < search_points_left.size(); i++) {

    if (!image_operator_.worldToImage(search_points_left[i], search_line.iStart)) {
      ROS_WARN_STREAM("[crosswalk detection] Unable to transform search segment start point "<<search_points_left[i]<<" to image frame, skipping segment");
      continue;
    }

    if (!image_operator_.worldToImage(search_points_right[i], search_line.iEnd)) {
      ROS_WARN_STREAM("[crosswalk detection] Unable to transform search segment end point "<<search_points_right[i]<<" to image frame, skipping segment");
      continue;
    }

    fixedLineOrdering(search_line, search_direction::y);

    image_operator_.findByLineSearch(search_line, *current_image_, search_direction::y, search_method::sobel, image_points, line_widths);
    ROS_DEBUG_STREAM("[crosswalk detection] Found "<<image_points.size()<<" points in image");

#if defined(DRAW_DEBUG) || defined(PUBLISH_DEBUG)
    cv::line(crosswalk_points_mat, search_line.iStart, search_line.iEnd, cv::Scalar(255,0,0), 1);
    for (auto point : image_points) {
      cv::circle(crosswalk_points_mat, point, 2, cv::Scalar(0,255,0));
    }
#endif
    if (image_points.size() > 5) {
      ROS_INFO_STREAM("[crosswalk detection] Found street crossing from "<<image_points.front()<<" to "<<image_points.back());
#if defined(DRAW_DEBUG) || defined(PUBLISH_DEBUG)
      cv::line(crosswalk_points_mat, image_points.front(), image_points.back(), cv::Scalar(0,255,0), 1);
#endif
    }
  }
#ifdef DRAW_DEBUG
  cv::imshow("Crosswalk detection", crosswalk_points_mat);
  cv::waitKey(1);
#endif
#ifdef PUBLISH_DEBUG
  visualization_pub_.publish(filtered_points_mat);
#endif
  return true;
}

} // namespace drive_ros_image_recognition

PLUGINLIB_EXPORT_CLASS(drive_ros_image_recognition::CrosswalkDetection, nodelet::Nodelet)
