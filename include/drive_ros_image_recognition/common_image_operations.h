#ifndef COMMON_IMAGE_OPERATIONS_H
#define COMMON_IMAGE_OPERATIONS_H

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <drive_ros_image_recognition/LineDetectionConfig.h>
#include <sensor_msgs/CameraInfo.h>
#include <drive_ros_msgs/Homography.h>

typedef boost::shared_ptr<cv::Mat> CvImagePtr;

inline CvImagePtr convertImageMessage(const sensor_msgs::ImageConstPtr& img_in) {
  CvImagePtr cv_ptr;
  try
  {
    // hardcopies for now, might be possible to process on pointer if fast enough
    // todo: make prettier
    //        cv_bridge::CvImagePtr temp_ptr = cv_bridge::toCvCopy(img_in, "");
    cv_ptr.reset(new cv::Mat(cv_bridge::toCvCopy(img_in, "")->image.clone()));
    //        cv_ptr.reset(new cv::Mat(temp_ptr->image.clone()));
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return NULL;
  }

  if( !cv_ptr->data)
  {
    ROS_WARN("Empty image received, skipping!");
    return NULL;
  }
  return cv_ptr;
}

namespace drive_ros_image_recognition {

inline void homography_callback(const drive_ros_msgs::HomographyConstPtr& homo_in, cv::Mat& cam2world, cv::Mat& world2cam,
                         bool& homography_received) {
  if (!homography_received)
    homography_received = true;

  ROS_ASSERT(homo_in->cam2world.layout.dim[0].size == 3 && homo_in->cam2world.layout.dim[1].size == 3 );
  cam2world = cv::Mat::zeros(3,3,CV_64FC1);
  int k=0;
  for (int i=0; i<homo_in->cam2world.layout.dim[0].size; i++){
    for (int j=0; j<homo_in->cam2world.layout.dim[1].size; j++){
      cam2world.at<double>(i,j) = homo_in->cam2world.data[k++];
    }
  }

  ROS_ASSERT(homo_in->world2cam.layout.dim[0].size == 3 && homo_in->world2cam.layout.dim[1].size == 3 );
  world2cam = cv::Mat::zeros(3,3,CV_64FC1);
  k=0;
  for (int i=0; i<homo_in->world2cam.layout.dim[0].size; i++){
    for (int j=0; j<homo_in->world2cam.layout.dim[1].size; j++){
      world2cam.at<double>(i,j) = homo_in->world2cam.data[k++];
    }
  }
}

inline void cam_info_sub(const sensor_msgs::CameraInfoConstPtr& incoming_cam_info, image_geometry::PinholeCameraModel& cam_model) {
  cam_model.fromCameraInfo(incoming_cam_info);
}

enum search_direction {
  x = false,
  y = true
};

enum search_method {
  sobel = false,
  brightness = true
};

struct SearchLine{
  cv::Point2f wStart;
  cv::Point2f wEnd;
  cv::Point2i iStart;
  cv::Point2i iEnd;

  cv::Point2f wLeft;
  cv::Point2f wMid;
  cv::Point2f wRight;
};

class TransformHelper {

public:

  TransformHelper():
    tf_listener_(),
    world2cam_(3,3,CV_64FC1,cv::Scalar(0.0)),
    cam_model_(),
    cam2world_(3,3,CV_64FC1,cv::Scalar(0.0)),
    cam_info_sub_(),
    homography_received_(false),
    homography_sub_()
  {
  }

  TransformHelper& operator=(TransformHelper&& other)
  {
    std::swap(cam_model_, other.cam_model_);
    std::swap(cam2world_, other.cam2world_);
    // tf listener is not swappable, should auto-initialize

    std::swap(world2cam_, other.world2cam_);
    // cannot swap subscriber, need to bind to new object
    cam_info_sub_ = ros::NodeHandle().subscribe<sensor_msgs::CameraInfo>("camera_info", 1,
                                                           boost::bind(&cam_info_sub, _1,
                                                                       getCameraModelReference()) );

    homography_sub_ = ros::NodeHandle().subscribe<drive_ros_msgs::Homography>("homography_in", 1,
                                                                           boost::bind(homography_callback, _1,
                                                                                       cam2world_, world2cam_, homography_received_));
    other.cam_info_sub_.shutdown();
    return *this;
  }

  // todo: properly reinitialize subscriber with new binding to this object
  TransformHelper(const TransformHelper& helper):
    tf_listener_()
  {
    cam2world_ = helper.cam2world_;
    world2cam_ = helper.world2cam_;
    cam_model_ = helper.cam_model_;
    cam_info_sub_ = ros::NodeHandle().subscribe<sensor_msgs::CameraInfo>("camera_info", 1,
                                                           boost::bind(&cam_info_sub, _1,
                                                                       getCameraModelReference()) );
    homography_sub_ = ros::NodeHandle().subscribe<drive_ros_msgs::Homography>("homography_in", 1,
                                                                           boost::bind(homography_callback, _1,
                                                                                       cam2world_, world2cam_, homography_received_));
  }

  TransformHelper(const image_geometry::PinholeCameraModel& cam_model):
    cam2world_(3,3,CV_64FC1,cv::Scalar(0.0)),
    world2cam_(3,3,CV_64FC1,cv::Scalar(0.0)),
    cam_model_(cam_model),
    tf_listener_(),
    cam_info_sub_()
  {
  }

  ~TransformHelper() {
  }

  bool init() {
    // subscribe Camera model to TransformHelper-> this is kind of hacky, but should keep the camera model on the transform helper updated
    cam_info_sub_ = ros::NodeHandle().subscribe<sensor_msgs::CameraInfo>("camera_info", 1,
                                                           boost::bind(&cam_info_sub, _1,
                                                                       getCameraModelReference()) );
    homography_sub_ = ros::NodeHandle().subscribe<drive_ros_msgs::Homography>("homography_in", 1,
                                                                           boost::bind(homography_callback, _1,
                                                                                       cam2world_, world2cam_, homography_received_));
  }

  void setCamModel(const image_geometry::PinholeCameraModel& cam_model) {
    cam_model_ = cam_model;
  }

  bool worldToImage(const cv::Point3f& world_point,
                    cv::Point& image_point) {
    // transform to image point and then repeat homography transform
    cv::Point2d pixel_coords = cam_model_.project3dToPixel(world_point);
    cv::Mat mat_pixel_coords(1,1,CV_64FC2,cv::Scalar(0.0));
    mat_pixel_coords.at<double>(0,0) = pixel_coords.x;
    mat_pixel_coords.at<double>(0,1) = pixel_coords.y;
    cv::Mat mat_image_point(1,1,CV_64FC2,cv::Scalar(0.0));
    cv::perspectiveTransform(mat_pixel_coords, mat_image_point, world2cam_);
    // todo: check if we lose precision here
    image_point = cv::Point(mat_image_point.at<double>(0,0), mat_image_point.at<double>(0,1));
    return true;
  }

  bool worldToImage(const cv::Point2f &world_point, cv::Point &image_point) {
    return worldToImage(cv::Point3f(world_point.x, world_point.y, 0.0f), image_point);
  }

  bool imageToWorld(const cv::Point& image_point, cv::Point2f& world_point) {
    cv::Mat mat_point_image(1,1,CV_64FC2,cv::Scalar(0.0));
    mat_point_image.at<double>(0,0) = image_point.x;
    mat_point_image.at<double>(0,1) = image_point.y;
    cv::Mat mat_point_world(1,1,CV_64FC2,cv::Scalar(0.0));
    cv::perspectiveTransform(mat_point_image, mat_point_world, cam2world_);
    cv::Point2f cam_point(mat_point_world.at<float>(0,0),mat_point_world.at<float>(0,1));

    // this returns a point with z=1, the pixel is on a ray from origin to this point
    // todo: set correct camera height and use it to calculate x and y of the world point
    double cam_height = 10.0;
    cv::Point3d cam_ray_point = cam_model_.projectPixelTo3dRay(cam_point);

    geometry_msgs::PointStamped camera_point;
    camera_point.header.frame_id = cam_model_.tfFrame();
    camera_point.point.x = cam_ray_point.x;
    camera_point.point.y = cam_ray_point.y;
    camera_point.point.z = cam_ray_point.z;
    geometry_msgs::PointStamped geom_world_point;

    try {
      ros::Duration timeout(1.0 / 30);
      tf_listener_.waitForTransform(cam_model_.tfFrame(), "tf_front_axis_middle",
                                    ros::Time::now(), timeout);
      tf_listener_.transformPoint("tf_front_axis_middle", camera_point, geom_world_point);
    }
    catch (tf::TransformException& ex) {
      ROS_WARN("[ImageToWorld] TF exception, unable to transform:\n%s", ex.what());
      return false;
    }
    world_point.x = geom_world_point.point.x;
    world_point.y = geom_world_point.point.y;
    return true;
  }

  void setcam2worldMat(const cv::Mat& cam2world) {
    cam2world_ = cam2world;
  }

  void setworld2camMat(const cv::Mat& world2cam) {
    world2cam_ = world2cam;
  }

  // to subscribe to camerainfo messages
  image_geometry::PinholeCameraModel& getCameraModelReference() {
    return cam_model_;
  }

private:
  image_geometry::PinholeCameraModel cam_model_;
  bool homography_received_;
  cv::Mat cam2world_;
  cv::Mat world2cam_;
  tf::TransformListener tf_listener_;
  ros::Subscriber cam_info_sub_;
  ros::Subscriber homography_sub_;
}; // class TransformHelper

class ImageOperator {

public:

  ImageOperator():
    helper_(),
    config_()
  {
    config_.sobelThreshold = 50;
    config_.brightness_threshold = 150;
  }

  ImageOperator(ImageOperator& image_operator):
    helper_(image_operator.helper_),
    config_(image_operator.config_)
  {
  }

  ImageOperator& operator=(ImageOperator&& other)
  {
    std::swap(helper_, other.helper_);
    std::swap(config_, other.config_);
    return *this;
  }

  ImageOperator(TransformHelper& helper):
    helper_(helper),
    config_()
  {
    config_.sobelThreshold = 50;
    config_.brightness_threshold = 150;
  }

  std::vector<cv::Point2f> returnValidPoints(const SearchLine &sl,
                                             const cv::Mat& current_image,
                                             const float lineWidth,
                                             const search_direction search_dir,
                                             const search_method method)
  {
    std::vector<cv::Point> imagePoints;
    std::vector<int> lineWidths;
    std::vector<cv::Point2f> foundPoints;

    float iDist = cv::norm(sl.iEnd - sl.iStart);
    float wDist = cv::norm(sl.wEnd - sl.wStart);
    float pxlPeakWidth = iDist/wDist*lineWidth;
    findByLineSearch(sl, current_image, search_dir, method, imagePoints, lineWidths);

    cv::Point2f wMid;

    for (int i=0; i<imagePoints.size(); ++i) {
      if (lineWidths[i] > pxlPeakWidth*config_.minLineWidthMul && lineWidths[i] < pxlPeakWidth*config_.maxLineWidthMul) {
        helper_.imageToWorld(imagePoints[i], wMid);
        foundPoints.push_back(wMid);
      }
    }
    return foundPoints;
  }

  void findByLineSearch(const SearchLine& sl,
                        const cv::Mat& current_image,
                        const search_direction search_dir,
                        const search_method search_meth,
                        std::vector<cv::Point>& image_points,
                        std::vector<int>& line_widths
                        )
  {
    cv::Mat processed_image;
    // step 0: cut image to line
    processed_image = current_image(cv::Rect(sl.iStart,
                                             cv::Point(sl.iEnd.x+(search_dir==search_direction::y),
                                                       sl.iEnd.y+(search_dir==search_direction::x))
                                             ));

    // step 1: threshold image with mat:
    if (search_meth == search_method::brightness) {
      cv::threshold(processed_image, processed_image, config_.brightness_threshold, 255, CV_THRESH_BINARY);
      // testing removal of bright white blobs
//      std::vector<std::vector<cv::Point> > image_contours;
//      cv::findContours(processed_image, image_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
//      processed_image = cv::Mat::zeros(current_image.rows, current_image.cols, CV_8UC1);
//      cv::Mat debug_exclude_image = cv::Mat::zeros(current_image.rows, current_image.cols, CV_8UC1);
//      for (int cnt = 0; cnt < image_contours.size(); ++cnt) {
//        if (cv::contourArea(image_contours[cnt]) < config_.blob_area_threshold)
//          cv::drawContours(processed_image, image_contours, cnt, cv::Scalar(255), CV_FILLED);
//        else if (cv::contourArea(image_contours[cnt])/cv::boundingRect(image_contours[cnt]).area()<config_.blob_aspect_threshold)
//          cv::drawContours(processed_image, image_contours, cnt, cv::Scalar(255), CV_FILLED);
//        else {
//          ROS_INFO_STREAM("Contour area of checked: "<<cv::contourArea(image_contours[cnt])<<" threshold is: "<<config_.blob_area_threshold);
//          ROS_INFO_STREAM("Aspect ratio of checked: "<<cv::contourArea(image_contours[cnt])/cv::boundingRect(image_contours[cnt]).area()<<" threshold is: "<<config_.blob_aspect_threshold);
//          cv::drawContours(debug_exclude_image, image_contours, cnt, cv::Scalar(255), CV_FILLED);
//        }
//        ROS_INFO_STREAM("Aspect ratio of checked: "<<cv::contourArea(image_contours[cnt])/cv::boundingRect(image_contours[cnt]).area()<<" threshold is: "<<config_.blob_aspect_threshold);
//      }
//      cv::namedWindow("Excluded contours", CV_WINDOW_NORMAL);
//      cv::imshow("Excluded contours", debug_exclude_image);
    } else if (search_meth == search_method::sobel) {
      processed_image.convertTo(processed_image, CV_16S);
      if (search_dir == search_direction::x) {
        cv::Sobel(processed_image, processed_image, CV_16S, 1, 0, 1, 1, 0, cv::BORDER_DEFAULT);
      }
      else if(search_dir == search_direction::y) {
        cv::Sobel(processed_image, processed_image, CV_16S, 0, 1, 1, 1, 0, cv::BORDER_DEFAULT);
      }
    }

#ifdef DRAW_DEBUG
    // display initial image
    cv::Mat debug_image = processed_image.clone();
    if (search_meth == search_method::brightness) {
      cv::namedWindow("Thresholded image in brightness search", CV_WINDOW_NORMAL);
      cv::imshow("Thresholded image in brightness search", debug_image);
    } else if (search_meth == search_method::sobel) {
      cv::namedWindow("Sobel image in line search", CV_WINDOW_NORMAL);
      cv::imshow("Sobel image in line search", debug_image);
    }
#endif

    if (search_meth == search_method::brightness) {
      // step3: get connected components (can be multiple) and calculate midpoints
      std::vector<std::vector<cv::Point> > line_contours;
      cv::findContours(processed_image, line_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

#ifdef DRAW_DEBUG
      // line contours
      cv::namedWindow("Line contours", CV_WINDOW_NORMAL);
      cv::Mat line_contour_image = processed_image.clone();
      for (int i=0; i<line_contours.size(); ++i)
        cv::drawContours(line_contour_image, line_contours, i, cv::Scalar(255));
      cv::imshow("Line contours", line_contour_image);
#endif

      // step5: get midpoints of components
      cv::Rect bounds;
      cv::Point temp_point;
      for (auto contour: line_contours) {
        bounds = cv::boundingRect( cv::Mat(contour) );
        // todo: add parameter if we will use contours
        if (bounds.area() > 6 && bounds.area() < 20) {
          line_widths.push_back(bounds.br().x-bounds.tl().x);
          temp_point = (bounds.tl()+bounds.br())/2;
          if (search_dir == search_direction::x) {
            temp_point.y = sl.iEnd.y;
          } else {
            temp_point.x = sl.iEnd.x;
          }
          image_points.push_back(temp_point);
        }
      }
    } else if (search_meth == search_method::sobel) {
      bool lowhigh_found = false;
      int first = 0;
      for(int i=0; i<processed_image.rows*processed_image.cols; ++i)
      {
        if (processed_image.at<short>(i) > config_.sobelThreshold) {
          lowhigh_found = true;
          first = i;
        } else if (lowhigh_found && processed_image.at<short>(i) < -config_.sobelThreshold) {
          line_widths.push_back(i-first);
          if (search_dir == search_direction::x) {
            image_points.push_back(cv::Point(sl.iStart.x+first+(i-first)/2,sl.iStart.y));
          }
          else if (search_dir == search_direction::y) {
            image_points.push_back(cv::Point(sl.iStart.x,sl.iStart.y+first+(i-first)/2));
          }
          lowhigh_found = false;
        }
      }
    }

    return;
  }

  // debug to perform line check
  void debugPointsImage (const cv::Mat& current_image,
                         const search_direction search_dir,
                         const search_method search_meth
                         ) {
    std::vector<cv::Point> image_points;
    std::vector<int> image_widths;
#ifdef DRAW_DEBUG
    cv::Mat debug_image;
    cv::cvtColor(current_image, debug_image, CV_GRAY2RGB);
#endif

    int search_steps = 6;
    int pixel_step = 50;
    SearchLine lines[search_steps];
    for (int it = 3; it < search_steps; ++it) {
      if (search_dir == search_direction::x) {
        lines[it].iStart = cv::Point(0,it*pixel_step);
        lines[it].iEnd = cv::Point(current_image.cols, it*pixel_step);
      } else if (search_dir == search_direction::y) {
        lines[it].iStart = cv::Point(it*pixel_step, 0);
        lines[it].iEnd = cv::Point(it*pixel_step, current_image.rows);
      }
#ifdef DRAW_DEBUG
      cv::line(debug_image, lines[it].iStart, lines[it].iEnd, cv::Scalar(255,255,255));
#endif
      findByLineSearch(lines[it],
                       current_image,
                       search_dir,
                       search_meth,
                       image_points,
                       image_widths
                       );
#ifdef DRAW_DEBUG
      for (auto point: image_points) {
        cv::circle(debug_image,point,2,cv::Scalar(0,255,0),2);
      }
#endif
    }
#ifdef DRAW_DEBUG
    if (search_meth == search_method::sobel) {
      cv::namedWindow("Debug lineCheck sobel", CV_WINDOW_NORMAL);
      cv::imshow("Debug lineCheck sobel", debug_image);
      cv::waitKey(1);
    }
    if (search_meth == search_method::brightness) {
      cv::namedWindow("Debug lineCheck brightness", CV_WINDOW_NORMAL);
      cv::imshow("Debug lineCheck brightness", debug_image);
      cv::waitKey(1);
    }
#endif
    return;
  }

  void setConfig(const LineDetectionConfig& config) {
    config_ = config;
  }

private:
  TransformHelper helper_;
  LineDetectionConfig config_;

}; // class ImageOperator

} // end namespace

#endif // COMMON_IMAGE_OPERATIONS_H
