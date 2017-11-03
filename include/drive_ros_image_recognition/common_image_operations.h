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
                                cv::Mat& scaling_mat, cv::Mat& scaling_mat_inv, bool& homography_received) {
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

  ROS_INFO_STREAM("Scaling mat: "<<scaling_mat);
  scaling_mat = world2cam*scaling_mat;
  scaling_mat_inv = scaling_mat.inv();
}

inline void camInfo_callback(const sensor_msgs::CameraInfoConstPtr& incoming_cam_info, image_geometry::PinholeCameraModel& cam_model, bool& cam_info_received) {
  sensor_msgs::CameraInfo incoming_clone = *incoming_cam_info;
  sensor_msgs::RegionOfInterest roi;
  roi.x_offset = 0;
  roi.width = 1280;
  // 344, cropped at image center
  // todo: remove this once all bags are updated to have this fixed
  roi.y_offset = 340;
  roi.height = 344;
  incoming_clone.roi = roi;
  cam_model.fromCameraInfo(incoming_clone);
  cam_info_received = true;
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

  TransformHelper(bool use_homography = true):
    tf_listener_(),
    world2cam_(3,3,CV_64FC1,cv::Scalar(0.0)),
    cam_model_(),
    cam2world_(3,3,CV_64FC1,cv::Scalar(0.0)),
    cam_info_sub_(),
    homography_received_(false),
    homography_sub_(),
    scaling_mat_(3,3,CV_64FC1,cv::Scalar(0.0)),
    scaling_mat_inv_(3,3,CV_64FC1,cv::Scalar(0.0)),
    transformed_size_(0,0),
    camera_model_received_(false),
    // temporary solution until have embedded correct frame in camerainfo messages (including the bag files)
    camera_frame_(""),
    world_frame_(""),
    use_homography_(use_homography)
  {
  }

  TransformHelper& operator=(TransformHelper&& other)
  {
    std::swap(cam_model_, other.cam_model_);
    std::swap(cam2world_, other.cam2world_);
    std::swap(camera_frame_, other.camera_frame_);
    std::swap(world_frame_, other.world_frame_);
    // tf listener is not swappable, should auto-initialize

    std::swap(world2cam_, other.world2cam_);
    std::swap(cam2world_, other.cam2world_);
    // cannot swap subscriber, need to bind to new object
    cam_info_sub_ = ros::NodeHandle().subscribe<sensor_msgs::CameraInfo>("camera_info", 1,
                                                           boost::bind(&camInfo_callback, _1,
                                                                       std::ref(cam_model_),
                                                                       std::ref(camera_model_received_) ) );

    homography_sub_ = ros::NodeHandle().subscribe<drive_ros_msgs::Homography>("homography_in", 1,
                                                                           boost::bind(homography_callback, _1,
                                                                                       std::ref(cam2world_), std::ref(world2cam_),
                                                                                       std::ref(scaling_mat_), std::ref(scaling_mat_inv_),
                                                                                       std::ref(homography_received_)));

    other.cam_info_sub_.shutdown();
    other.homography_sub_.shutdown();
    return *this;
  }

  // todo: properly reinitialize subscriber with new binding to this object
  TransformHelper(const TransformHelper& helper):
    tf_listener_()
  {
    cam2world_ = helper.cam2world_;
    world2cam_ = helper.world2cam_;
    cam_model_ = helper.cam_model_;
    world_frame_ = helper.world_frame_;
    camera_frame_ = helper.camera_frame_;
    cam_info_sub_ = ros::NodeHandle().subscribe<sensor_msgs::CameraInfo>("camera_info", 1,
                                                           boost::bind(&camInfo_callback, _1,
                                                                       std::ref(cam_model_),
                                                                       std::ref(camera_model_received_)));
    homography_sub_ = ros::NodeHandle().subscribe<drive_ros_msgs::Homography>("homography_in", 1,
                                                                           boost::bind(homography_callback, _1,
                                                                                       std::ref(cam2world_), std::ref(world2cam_),
                                                                                       std::ref(scaling_mat_), std::ref(scaling_mat_inv_),
                                                                                       std::ref(homography_received_)));
  }

  TransformHelper(const image_geometry::PinholeCameraModel& cam_model):
    cam2world_(3,3,CV_64FC1,cv::Scalar(0.0)),
    world2cam_(3,3,CV_64FC1,cv::Scalar(0.0)),
    scaling_mat_(3,3,CV_64FC1,cv::Scalar(0.0)),
    scaling_mat_inv_(3,3,CV_64FC1,cv::Scalar(0.0)),
    transformed_size_(0,0),
    cam_model_(cam_model),
    tf_listener_(),
    cam_info_sub_(),
    camera_model_received_(false),
    // temporary solution until have embedded correct frame in camerainfo messages (including the bag files)
    camera_frame_(""),
    world_frame_(""),
    use_homography_(true)
  {
  }

  ~TransformHelper() {
  }

  bool init() {
    ros::NodeHandle pnh = ros::NodeHandle("~");
    std::vector<double> world_size;
    if(!pnh.getParam("world_size", world_size)) {
       ROS_ERROR("Unable to load parameter world_size!");
       return false;
    }
    ROS_ASSERT(world_size.size() == 2);
    std::vector<double> image_size;
    if(!pnh.getParam("image_size", image_size)) {
      ROS_ERROR("Unable to load parameter world_size!");
      return false;
    }
    ROS_ASSERT(image_size.size() == 2);
    transformed_size_ = cv::Size(image_size[0],image_size[1]);
    scaling_mat_.at<double>(0,0) = world_size[0]/image_size[0];
    scaling_mat_.at<double>(1,1) = -world_size[1]/image_size[1];
    scaling_mat_.at<double>(1,2) = world_size[1]/2;
    scaling_mat_.at<double>(2,2) = 1.0;

    // subscribe Camera model to TransformHelper-> this is kind of hacky, but should keep the camera model on the transform helper updated
    cam_info_sub_ = ros::NodeHandle().subscribe<sensor_msgs::CameraInfo>("camera_info", 1,
                                                           boost::bind(&camInfo_callback, _1,
                                                                       std::ref(cam_model_),
                                                                       std::ref(camera_model_received_) ) );
    homography_sub_ = ros::NodeHandle().subscribe<drive_ros_msgs::Homography>("homography_in", 1,
                                                                           boost::bind(homography_callback, _1,
                                                                                       std::ref(cam2world_), std::ref(world2cam_),
                                                                                       std::ref(scaling_mat_), std::ref(scaling_mat_inv_),
                                                                                       std::ref(homography_received_)));
  }

  void setCamModel(const image_geometry::PinholeCameraModel& cam_model) {
    cam_model_ = cam_model;
  }

  void setWorldFrame(const std::string& frame) {
    if (camera_frame_ != std::string("")) {
      try {
        ros::Duration timeout(1.0);
        tf_listener_.waitForTransform(camera_frame_, frame,
                                      ros::Time::now(), timeout);
      }
      catch (tf::TransformException& ex) {
        ROS_WARN("[TransformHelper] TF exception, unable to find transform to new camera frame %s, will not set it, reason:\n%s", frame.c_str(), ex.what());
        return;
      }
    }
    world_frame_ = frame;
  }

  // placeholder until we get updated bags with camera messages
  // cannot access PinHoleCameraModel after it has been created, so just use a string instead
  void setCameraFrame(const std::string& frame) {
    if (world_frame_ != std::string("")) {
      try {
        ros::Duration timeout(1.0);
        tf_listener_.waitForTransform(world_frame_, frame,
                                      ros::Time::now(), timeout);
      }
      catch (tf::TransformException& ex) {
        ROS_WARN("[TransformHelper] TF exception, unable to find transform to new world frame %s, will not set it, reason:\n%s", frame.c_str(), ex.what());
        return;
      }
    }
    camera_frame_ = frame;
  }

  bool worldToImage(const cv::Point3f& world_point,
                    cv::Point& image_point) {

    if(!homography_received_ || !camera_model_received_) {
      if (!homography_received_)
        ROS_WARN_STREAM("[WorldToImage] Homography not received yet, skipping world to image transformation requested for world point "<<world_point<<" and returning image Point2i (0,0)");
      if (!camera_model_received_)
        ROS_WARN_STREAM("[WorldToImage] Camera model not received yet, skipping world to image transformation requested for world point "<<world_point<<" and returning image Point2i (0,0)");
      image_point = cv::Point(0,0);
      return false;
    }

    // transform to image point and then repeat homography transform
    cv::Point2d pixel_coords = cam_model_.project3dToPixel(world_point);
    cv::Mat mat_pixel_coords(1,1,CV_64FC2,cv::Scalar(0.0));
    mat_pixel_coords.at<double>(0,0) = pixel_coords.x;
    mat_pixel_coords.at<double>(0,1) = pixel_coords.y;
    cv::Mat mat_image_point(1,1,CV_64FC2,cv::Scalar(0.0));
    if (use_homography_) {
      cv::perspectiveTransform(mat_pixel_coords, mat_image_point, scaling_mat_inv_);
      image_point = cv::Point(mat_image_point.at<double>(0,0), mat_image_point.at<double>(0,1));
    }
    else {
      // nothing to transform if homography has not been applied
      image_point = cv::Point(pixel_coords.x, pixel_coords.y);
    }
    return true;
  }

  bool worldToImage(const cv::Point2f &world_point, cv::Point &image_point) {
    return worldToImage(cv::Point3f(world_point.x, world_point.y, 0.0f), image_point);
  }

  bool imageToWorld(const cv::Point& image_point, cv::Point2f& world_point, std::string target_frame = std::string("/rear_axis_middle_ground"), cv::Mat test_image = cv::Mat()) {
    if(!homography_received_ || !camera_model_received_) {
      if (!homography_received_)
        ROS_WARN_STREAM("[ImageToWorld] Homography not received yet, skipping image to world transformation requested for image point "<<image_point<<" and returning world Point2f (0.0,0.0)");
      if (!camera_model_received_)
        ROS_WARN_STREAM("[ImageToWorld] Camera model not received yet, skipping image to world transformation requested for image point "<<image_point<<" and returning world Point2f (0.0,0.0)");
      world_point = cv::Point2f(0.0,0.0);
      return false;
    }

    std::vector<cv::Point2f> obj_corners(1);
    obj_corners[0] = cv::Point2f(image_point.x, image_point.y);
    std::vector<cv::Point2f> scene_corners(1);
    if (homography_received_)
      cv::perspectiveTransform(obj_corners, scene_corners, scaling_mat_);
    else
      scene_corners[0] = obj_corners[0]; // no homography transforms if we use camera image directly
    // this returns a point with z=1, the pixel is on a ray from origin to this point
    cv::Point3d cam_ray_point = cam_model_.projectPixelTo3dRay(scene_corners[0]);

    // this gives us a ray defined by two points, now we transform to the fixed frame
    // where we can easily define the road plane and intersect the ray with it to get the world point
    geometry_msgs::PointStamped camera_point;
    camera_point.header.frame_id = camera_frame_;
    camera_point.point.x = cam_ray_point.x;
    camera_point.point.y = cam_ray_point.y;
    camera_point.point.z = cam_ray_point.z;
    geometry_msgs::PointStamped camera_point_world;
    tf::StampedTransform transform;
    try {
      tf_listener_.transformPoint(target_frame, camera_point, camera_point_world);
      tf_listener_.lookupTransform(target_frame, camera_frame_,
                                   ros::Time::now(), transform);
    }
    catch (tf::TransformException& ex) {
      ROS_WARN("[ImageToWorld] TF exception, unable to transform:\n%s", ex.what());
      return false;
    }

    // ray-plane intersection (plane is defined by P=[0 0 0] and n=[0 0 1]
    tf::Point ray_point_tf;
    tf::Vector3 plane_normal(0,0,1);
    tf::Vector3 plane_point(0,0,0);
    tf::pointMsgToTF(camera_point_world.point, ray_point_tf);
    tf::Vector3 ray_dir = ray_point_tf - transform.getOrigin();
    double denom = plane_normal.dot(ray_dir);

    if (std::abs(denom) > 1e-9) {
      double scale_axis = (plane_point-transform.getOrigin()).dot(plane_normal)/denom;
      tf::Vector3 return_point = transform.getOrigin()+scale_axis*ray_dir;
      world_point.x = return_point.x();
      world_point.y = return_point.y();
      return true;
    }
    else {
      ROS_WARN_STREAM("[imageToWorld] Camera ray and ground plane are parallel, unable to transform image point "<<image_point);
      return false;
    }
  }

  void setcam2worldMat(const cv::Mat& cam2world) {
    cam2world_ = cam2world;
  }

  void setworld2camMat(const cv::Mat& world2cam) {
    world2cam_ = world2cam;
  }

private:
  image_geometry::PinholeCameraModel cam_model_;
  bool use_homography_;
  bool homography_received_;
  bool camera_model_received_;
  std::string world_frame_;
  // temporary solution until have embedded correct frame in camerainfo messages (including the bag files)
  std::string camera_frame_;
  cv::Mat cam2world_;
  cv::Mat world2cam_;
  cv::Mat scaling_mat_;
  cv::Mat scaling_mat_inv_;
  cv::Size transformed_size_;
  tf::TransformListener tf_listener_;
  ros::Subscriber cam_info_sub_;
  ros::Subscriber homography_sub_;
}; // class TransformHelper

class ImageOperator : public TransformHelper {

public:

  ImageOperator():
    TransformHelper(),
    config_()
  {
    config_.sobelThreshold = 50;
    config_.brightness_threshold = 150;
  }

  ImageOperator(ImageOperator& image_operator):
    TransformHelper(),
    config_(image_operator.config_)
  {
  }

  ImageOperator& operator=(ImageOperator&& other)
  {
    std::swap(config_, other.config_);
    return *this;
  }

  ImageOperator(TransformHelper& helper):
    TransformHelper(helper),
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
        imageToWorld(imagePoints[i], wMid);
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
  LineDetectionConfig config_;

}; // class ImageOperator

} // end namespace

#endif // COMMON_IMAGE_OPERATIONS_H
