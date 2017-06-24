#include <drive_ros_image_recognition/warp_image.h>
#include <pluginlib/class_list_macros.h>

namespace drive_ros_image_recognition {

WarpContent::WarpContent(const ros::NodeHandle& pnh):
  pnh_(pnh),
  current_image_(),
  world2cam_(3,3,CV_64F,cv::Scalar(0.0)),
  cam2world_(3,3,CV_64F,cv::Scalar(0.0)),
  cam_mat_(3,3,CV_64F,cv::Scalar(0.0)),
  dist_coeffs_(8,1,CV_64F,cv::Scalar(0.0)),
  it_(pnh),
  cam_sub_(),
  worldToImageServer_(),
  imageToWorldServer_(),
  cam_model_(),
  tf_listener_()
{
  img_pub_ = it_.advertise("warped_out", 1);
  undistort_pub_ = it_.advertise("undistort_out", 1);
}

WarpContent::~WarpContent() {
}

bool WarpContent::init() {
  // retrieve world2map and map2world homography matrices
  std::vector<double> temp_vals;

  if (!pnh_.getParam("homography_matrix/world2cam", temp_vals)) {
    ROS_ERROR("Unable to load world2cam homography matrix from configuration file!");
    return false;
  }
  if (temp_vals.size() != 9) {
    ROS_ERROR("Retreived world2cam homography matrix does not have 9 values!");
    return false;
  }
  for(unsigned i=0; i < temp_vals.size(); i++) {
    // todo: check: according to warp_cpp.h, those two might actually be the other way round
    world2cam_.at<double>(i) = temp_vals[i];
  }
  ROS_DEBUG_STREAM("World2cam loaded as: "<<world2cam_);

  temp_vals.clear();
  if (!pnh_.getParam("homography_matrix/cam2world", temp_vals)) {
    ROS_ERROR("Unable to load cam2world homography matrix from configuration file!");
    return false;
  }
  if (temp_vals.size() != 9) {
    ROS_ERROR("Retreived cam2world homography matrix does not have 9 values!");
    return false;
  }
  for(unsigned i=0; i < temp_vals.size(); i++) {
    // todo: check: according to warp_cpp.h, those two might actually be the other way round
    cam2world_.at<double>(i) = temp_vals[i];
  }
  ROS_DEBUG_STREAM("Cam2World loaded as: "<<cam2world_);

  // retreive camera model matrix for undistortion
  temp_vals.clear();
  if (!pnh_.getParam("camera_matrix/cam_mat", temp_vals)) {
    ROS_ERROR("Unable to load camera matrix parameters from configuration file!");
    return false;
  }
  if (temp_vals.size() != 4) {
    ROS_ERROR("Retreived camera matrix does not have 4 values!");
    return false;
  }
  // (row,column) indexing
  // Fx:
  cam_mat_.at<double>(0,0) = temp_vals[0];
  // Fy:
  cam_mat_.at<double>(1,1) = temp_vals[1];
  // Cx:
  cam_mat_.at<double>(0,2) = temp_vals[2];
  // Cy:
  cam_mat_.at<double>(1,2) = temp_vals[3];
  cam_mat_.at<double>(2,2) = 1.0;
  ROS_DEBUG_STREAM("Camera matrix loaded as: "<<cam_mat_);

  // retreive distortion parameter vector for undistortion
  temp_vals.clear();
  if (!pnh_.getParam("camera_matrix/dist_coeffs", temp_vals)) {
    ROS_ERROR("Unable to load distortion coefficient vector parameters from configuration file!");
    return false;
  }
  if (temp_vals.size() != 6) {
    ROS_ERROR("Retreived distortion coefficient vector does not have 6 values!");
    return false;
  }
  // (row,column) indexing
  // K1
  dist_coeffs_.at<double>(0,0) = temp_vals[0];
  // K2
  dist_coeffs_.at<double>(1,0) = temp_vals[1];
  // K3
  dist_coeffs_.at<double>(4,0) = temp_vals[2];
  // K4
  dist_coeffs_.at<double>(5,0) = temp_vals[3];
  // K5
  dist_coeffs_.at<double>(6,0) = temp_vals[4];
  // K6
  dist_coeffs_.at<double>(7,0) = temp_vals[5];
  ROS_DEBUG_STREAM("Distortion coefficients loaded as: "<<dist_coeffs_);

  // initialize homography transformation subscriber
  cam_sub_ = it_.subscribeCamera("img_in", 10, &WarpContent::world_image_callback, this);
  worldToImageServer_ = pnh_.advertiseService("WorldToImage", &WarpContent::worldToImage, this);
  imageToWorldServer_ = pnh_.advertiseService("ImageToWorld", &WarpContent::imageToWorld, this);
  return true;
}

void WarpContent::world_image_callback(const sensor_msgs::ImageConstPtr& msg,
                                       const sensor_msgs::CameraInfoConstPtr& info_msg) {
  try
  {
    // copy
    current_image_ = cv_bridge::toCvCopy(msg, "")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_WARN("Could not convert incoming image from '%s' to 'CV_16U', skipping.", msg->encoding.c_str());
    return;
  }

    cam_model_.fromCameraInfo(info_msg);

  // optionally: scale homography, as image is too small
//  cv::Rect roi(cv::Point(current_image_.cols/2-(410/2),0),cv::Point(current_image_.cols/2+(410/2),current_image_.rows));
//  current_image_ = current_image_(roi);
//  cv::Mat S = cv::Mat::eye(3,3,CV_64F);
//  S.at<double>(0,0) = 410/current_image_.rows;
//  S.at<double>(1,1) = 752/current_image_.cols;

  // undistort and apply homography transformation
  cv::Mat undistorted_mat;
  cv::undistort(current_image_, undistorted_mat, cam_model_.fullProjectionMatrix(), cam_model_.distortionCoeffs());
  undistort_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_8UC1, undistorted_mat).toImageMsg());
  // opionally: apply scaled homography
  //  cv::warpPerspective(current_image_, current_image_, S*world2cam_*S.inv(), current_image_.size(),cv::WARP_INVERSE_MAP);

  // flag ensures that we directly use the matrix, as it is done in LMS
//  cv::warpPerspective(undistorted_mat, undistorted_mat, world2cam_, current_image_.size(),cv::WARP_INVERSE_MAP);
//  img_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_8UC1, undistorted_mat).toImageMsg());
}

bool WarpContent::worldToImage(drive_ros_image_recognition::WorldToImage::Request &req,
                               drive_ros_image_recognition::WorldToImage::Response &res)
{
//  ROS_DEBUG("WorldToImage service: transforming incoming world coordinates to image coordinates");
//  cv::Mat point_world(1,1,CV_64FC2);
//  point_world.at<double>(0,0) = req.world_point.x;
//  point_world.at<double>(0,1) = req.world_point.y;
//  point_world.at<double>(0,2) = req.world_point.z;

  cv::Point3d world_point(req.world_point.point.x, req.world_point.point.y, req.world_point.point.z);
  cv::Point2d world_point_cam = cam_model_.project3dToPixel(world_point);

//  cv::Mat point_image(1,1,CV_64FC2);
//  cv::perspectiveTransform(point_world, point_image, world2cam_);
//  if (point_world.rows != 1 || point_world.cols != 1) {
//    ROS_WARN("Point transformed to image dimensions has invalid dimensions");
//    return false;
//  }

//  res.image_point.x = point_image.at<double>(0,0);
//  res.image_point.y = point_image.at<double>(0,1);
//  res.image_point.z = 0.0;

  res.image_point.x = world_point_cam.x;
  res.image_point.y = world_point_cam.y;
  res.image_point.z = 0.0;

  return true;
}

bool WarpContent::imageToWorld(drive_ros_image_recognition::ImageToWorld::Request  &req,
                               drive_ros_image_recognition::ImageToWorld::Response &res)
{
//  ROS_DEBUG("WorldToImage service: transforming incoming image coordinates to world coordinates");
//  cv::Mat point_world(1,1,CV_64FC2);
//  point_world.at<double>(0,0) = req.image_point.x;
//  point_world.at<double>(0,1) = req.image_point.y;

//  cv::Mat point_image(1,1,CV_64FC2);
//  cv::perspectiveTransform(point_world, point_image, cam2world_);
//  if (point_world.rows != 1 || point_world.cols != 1) {
//    ROS_WARN("Point transformed to world dimensions has invalid dimensions");
//    return false;
//  }
//  res.world_point.x = point_image.at<double>(0,0);
//  res.world_point.y = point_image.at<double>(0,1);
//  res.world_point.z = point_image.at<double>(0,2);
//  // todo: move to sensor_msgs::CameraInfo for transformation, uses tf as base, could use as unified interface
//  return true;

  // assuming we are defining our road points in the middle from axis frame id
//  geometry_msgs::PointStamped point_out;
//  cv::Point2d image_point_;
//  cv::Point3d world_point_;
//  image_point_.x = req.image_point.x;
//  image_point_.y = req.image_point.y;
//  world_point_ = cam_model_.projectPixelTo3dRay(image_point_);
//  cam_model_.
    cv::Mat point_world(1,1,CV_64FC2);
    point_world.at<double>(0,0) = req.image_point.x;
    point_world.at<double>(0,1) = req.image_point.y;

    cv::Mat point_image(1,1,CV_64FC2);
    cv::perspectiveTransform(point_world, point_image, cam2world_);
    if (point_world.rows != 1 || point_world.cols != 1) {
        ROS_WARN("Point transformed to world dimensions has invalid dimensions");
        return false;
    }

    // todo: check if this is valid
    cv::Point3d cam_ray_point = cam_model_.projectPixelTo3dRay(cv::Point2d(point_image.at<double>(0,0),point_image.at<double>(0,1)));

    geometry_msgs::PointStamped camera_point;
    camera_point.header.frame_id = cam_model_.tfFrame();
    camera_point.point.x = cam_ray_point.x;
    camera_point.point.y = cam_ray_point.y;
    camera_point.point.z = cam_ray_point.z;

    try {
        ros::Duration timeout(1.0 / 30);
        tf_listener_.waitForTransform(cam_model_.tfFrame(), cam_model_.tfFrame(),
                                      ros::Time::now(), timeout);
        tf_listener_.transformPoint("tf_front_axis_middle", camera_point, res.world_point);
    }
    catch (tf::TransformException& ex) {
        ROS_WARN("[ImageToWorld] TF exception, unable to transform:\n%s", ex.what());
        return false;
    }
    return true;
}

void WarpImageNodelet::onInit()
{
  my_content_.reset(new WarpContent(ros::NodeHandle(getPrivateNodeHandle())));
  if (!my_content_->init()) {
    ROS_ERROR("WarpImageNodelet failed to initialize!");
    // nodelet failing will kill the entire loader anyway
    ros::shutdown();
  }
  else {
    ROS_INFO("Warp_image nodelet succesfully initialized");
  }
}

} // namespace drive_ros_image_recognition

PLUGINLIB_EXPORT_CLASS(drive_ros_image_recognition::WarpImageNodelet, nodelet::Nodelet)
