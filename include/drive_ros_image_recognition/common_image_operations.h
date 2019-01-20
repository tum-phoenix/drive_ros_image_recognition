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
                                cv::Mat& scaling_mat, cv::Mat& scaling_mat_inv, cv::Mat& scaledCam2World, cv::Mat& scaledWorld2Cam,
                                bool& homography_received) {
  if (homography_received) {
    // the homography will not change during runtime
    return;
  }

  homography_received = true;

  // extract cam2world matrix from the homography message
  ROS_ASSERT(homo_in->cam2world.layout.dim[0].size == 3 && homo_in->cam2world.layout.dim[1].size == 3 );
  cam2world = cv::Mat::zeros(3,3,CV_64FC1);
  int k=0;
  for (int i=0; i<homo_in->cam2world.layout.dim[0].size; i++){
    for (int j=0; j<homo_in->cam2world.layout.dim[1].size; j++){
      cam2world.at<double>(i,j) = homo_in->cam2world.data[k++];
    }
  }

  // extract world2cam matrix from the homography message
  ROS_ASSERT(homo_in->world2cam.layout.dim[0].size == 3 && homo_in->world2cam.layout.dim[1].size == 3 );
  world2cam = cv::Mat::zeros(3,3,CV_64FC1);
  k=0;
  for (int i=0; i<homo_in->world2cam.layout.dim[0].size; i++){
    for (int j=0; j<homo_in->world2cam.layout.dim[1].size; j++){
      world2cam.at<double>(i,j) = homo_in->world2cam.data[k++];
    }
  }

  // we need the scaled matrices for the warped image (bird eye view)
  scaling_mat_inv = scaling_mat.inv();
  scaledCam2World = scaling_mat_inv * cam2world;
  scaledWorld2Cam = world2cam * scaling_mat;

  ROS_INFO_STREAM("Scaling mat:\n" << scaling_mat);
  ROS_INFO_STREAM("received cam2world:\n" << cam2world);
  ROS_INFO_STREAM("received world2cam:\n" << world2cam);
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

class TransformHelper {

public:
  TransformHelper():
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
    world_frame_("")
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
                                                                                       std::ref(scaledCam2world_), std::ref(scaledWorld2cam_),
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
                                                                                       std::ref(scaledCam2world_), std::ref(scaledWorld2cam_),
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
    world_frame_("")
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
    ROS_INFO("TransformHelper subscribes to '%s' for camera_info", cam_info_sub_.getTopic().c_str());

    homography_sub_ = ros::NodeHandle().subscribe<drive_ros_msgs::Homography>("homography_in", 1,
                                                                           boost::bind(homography_callback, _1,
                                                                                       std::ref(cam2world_), std::ref(world2cam_),
                                                                                       std::ref(scaling_mat_), std::ref(scaling_mat_inv_),
                                                                                       std::ref(scaledCam2world_), std::ref(scaledWorld2cam_),
                                                                                       std::ref(homography_received_)));
    ROS_INFO("TransformHelper subscribes to '%s' for homography", homography_sub_.getTopic().c_str());
    return true;
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

  ///
  /// \brief imageToWorld
  /// Converts all imagePoints to the corresponding worldPoints using the cam2world matrix.
  /// \param imagePoints
  /// \param worldPoints
  /// \return true on success, false on failure.
  ///
  bool imageToWorld(std::vector<cv::Point2f> &imagePoints, std::vector<cv::Point2f> &worldPoints) {
    if(imagePoints.empty()) {
      return false;
    }
    if(!homography_received_) {
      ROS_WARN_STREAM("[imagetoWorld] Homography not received yet");
      return false;
    }
    cv::perspectiveTransform(imagePoints, worldPoints, cam2world_);
    return true;
  }

  ///
  /// \brief worldToImage
  /// Converts all worldPoints to the corresponding imagePoints using the world2cam matrix.
  /// \param worldPoints
  /// \param imagePoints
  /// \return true on success, false on failure.
  ///
  bool worldToImage(std::vector<cv::Point2f> &worldPoints, std::vector<cv::Point2f> &imagePoints) {
    if(worldPoints.empty()) {
      return false;
    }
    if(!homography_received_) {
      ROS_WARN_STREAM("[worldToImage] Homography not received yet");
      return false;
    }
    cv::perspectiveTransform(worldPoints, imagePoints, world2cam_);
    return true;
  }

  ///
  /// \brief homographImage
  /// Warps the srcImg to get the "bird eye view" image.
  /// \param srcImg
  /// \param dstImg
  /// \return true on success, false on failure.
  ///
  bool homographImage(cv::Mat &srcImg, cv::Mat &dstImg) {
    if(!homography_received_) {
       ROS_WARN_STREAM("[homographImage] Homography not received yet");
      return false;
    }
    dstImg = cv::Mat::zeros( srcImg.rows, srcImg.cols, srcImg.type() );
    cv::warpPerspective(srcImg, dstImg, scaledCam2world_, transformed_size_);
    return true;
  }

  ///
  /// \brief cameraToWarpedImg
  /// Converts a camera points to the corresponding points in the warped image.
  /// \param cameraPoints
  /// \param worldPoint
  /// \return true on success, false on failure.
  ///
  bool cameraToWarpedImg(std::vector<cv::Point2f> &cameraPoints, std::vector<cv::Point2f> &warpedImgPoints) {
    if(!homography_received_) {
      ROS_WARN_STREAM("[cameraToWarpedImg] Homography not received yet");
      return false;
    }
    cv::perspectiveTransform(cameraPoints, warpedImgPoints, scaledCam2world_);
    return true;
  }

  bool warpedImgToCamera(std::vector<cv::Point2f> &warpedImgPoints, std::vector<cv::Point2f> &cameraPoints) {
      if(!homography_received_) {
        ROS_WARN_STREAM("[warpedImgToCamera] Homography not received yet");
        return false;
      }
      cv::perspectiveTransform(warpedImgPoints, cameraPoints, scaledWorld2cam_);
      return true;
  }

  bool worldToWarpedImg(std::vector<cv::Point2f> &worldPts, std::vector<cv::Point2f> &warpedImgPts) {
      std::vector<cv::Point2f> tmp;
      worldToImage(worldPts, tmp);
      cameraToWarpedImg(tmp, warpedImgPts);
      return true;
  }

  bool warpedImgToWorld(std::vector<cv::Point2f> &warpedImgPts, std::vector<cv::Point2f> &worldPts) {
      std::vector<cv::Point2f> tmp;
      warpedImgToCamera(warpedImgPts, tmp);
      imageToWorld(tmp, worldPts);
      return true;
  }

  /*
   * Simon says: these two methods are not working properly, since we are doing the homography twice (first use the homog matrix
   * and then the TF conversion. I will leave this in, in case we need the tf version later.
   */
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

  bool worldToImage(const cv::Point2f& world_point, cv::Point2f& image_point) {
    if (!homography_received_) {
      ROS_WARN_STREAM("[WorldToImage] Homography not received yet, skipping world to image transformation requested for world point "<<world_point<<" and returning image Point2i (0,0)");
      image_point = cv::Point(0,0);
      return false;
    }

    if (!camera_model_received_) {
      ROS_WARN_STREAM("[WorldToImage] Camera model not received yet, skipping world to image transformation requested for world point "<<world_point<<" and returning image Point2i (0,0)");
      image_point = cv::Point(0,0);
      return false;
    }

    std::vector<cv::Point2f> worldPoints(1);
    std::vector<cv::Point2f> imagePoints(1);
    worldPoints[0] = cv::Point2f(image_point.x, image_point.y);
    cv::perspectiveTransform(worldPoints, imagePoints, world2cam_);
    image_point = imagePoints.at(0);

    // transform to image point and then repeat homography transform
    cv::Point2d pixel_coords = cam_model_.project3dToPixel(cv::Point3f(world_point.x, world_point.y, 0.0));
    cv::Mat mat_pixel_coords(1,1,CV_64FC2,cv::Scalar(0.0));
    mat_pixel_coords.at<double>(0,0) = pixel_coords.x;
    mat_pixel_coords.at<double>(0,1) = pixel_coords.y;
    cv::Mat mat_image_point(1,1,CV_64FC2,cv::Scalar(0.0));
    cv::perspectiveTransform(mat_pixel_coords, mat_image_point, scaling_mat_);
    image_point = cv::Point(mat_image_point.at<double>(0,0), mat_image_point.at<double>(0,1));


    return true;
  }

private:
  image_geometry::PinholeCameraModel cam_model_;
  bool homography_received_;
  bool camera_model_received_;
  std::string world_frame_;
  // temporary solution until have embedded correct frame in camerainfo messages (including the bag files)
  std::string camera_frame_;
  cv::Mat cam2world_;
  cv::Mat world2cam_;
  cv::Mat scaling_mat_;
  cv::Mat scaling_mat_inv_;
  cv::Mat scaledWorld2cam_;
  cv::Mat scaledCam2world_;
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
  }

  void setConfig(const LineDetectionConfig& config) {
    config_ = config;
  }

private:
  LineDetectionConfig config_;

}; // class ImageOperator

} // end namespace

#endif // COMMON_IMAGE_OPERATIONS_H
