#include <drive_ros_image_recognition/warp_image.h>

namespace drive_ros_image_recognition {

WarpContent::WarpContent(ros::NodeHandle& pnh):
  pnh_(pnh),
  current_image_(),
  world2cam_(3,3,CV_64F,cv::Scalar(0.0)),
  cam2world_(3,3,CV_64F,cv::Scalar(0.0)),
  cam_mat_(3,3,CV_64F,cv::Scalar(0.0)),
  dist_coeffs_(8,1,CV_64F,cv::Scalar(0.0)),
  it_(pnh),
  img_sub_(),
  worldToImageServer_(),
  imageToWorldServer_()
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
  // todo: if the index does not match opencv default
//  world2cam_.at<double>(0,0) = temp_vals[0];
//  world2cam_.at<double>(1,0) = temp_vals[1];
//  world2cam_.at<double>(2,0) = temp_vals[2];
//  world2cam_.at<double>(0,1) = temp_vals[3];
//  world2cam_.at<double>(1,1) = temp_vals[4];
//  world2cam_.at<double>(2,1) = temp_vals[5];
//  world2cam_.at<double>(0,2) = temp_vals[6];
//  world2cam_.at<double>(1,2) = temp_vals[7];
//  world2cam_.at<double>(2,2) = temp_vals[8];
  ROS_INFO_STREAM("World2cam loaded as: "<<world2cam_);

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
  ROS_INFO_STREAM("Cam2World loaded as: "<<cam2world_);

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
  ROS_INFO_STREAM("Camera matrix loaded as: "<<cam_mat_);

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
  ROS_INFO_STREAM("Distortion coefficients loaded as: "<<dist_coeffs_);

  // initialize homography transformation subscriber
  img_sub_ = it_.subscribe("img_in", 10, &WarpContent::world_image_callback, this);
  worldToImageServer_ = pnh_.advertiseService("WorldToImage", &WarpContent::worldToImage, this);
  imageToWorldServer_ = pnh_.advertiseService("ImageToWorld", &WarpContent::imageToWorld, this);
  return true;
}

void WarpContent::world_image_callback(const sensor_msgs::ImageConstPtr& msg) {
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
  // todo: extract required region from image, defined in the config, should be done in the camera

  // undistort and apply homography transformation
  cv::Mat undistorted_mat;
  cv::undistort(current_image_, undistorted_mat, cam_mat_, dist_coeffs_);
  undistort_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_8UC1, undistorted_mat).toImageMsg());
  cv::warpPerspective(undistorted_mat, undistorted_mat, world2cam_, current_image_.size());
  img_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_8UC1, undistorted_mat).toImageMsg());
}

bool WarpContent::worldToImage(drive_ros_image_recognition::WorldToImage::Request &req,
                               drive_ros_image_recognition::WorldToImage::Response &res)
{
  ROS_DEBUG("WorldToImage service: transforming incoming world coordinates to image coordinates");
  cv::Mat point_world = cv::Mat::zeros(3,1,CV_64F);
  point_world.at<double>(0,0) = req.world_point.x;
  point_world.at<double>(1,0) = req.world_point.y;
  point_world.at<double>(2,0) = req.world_point.z;

  cv::Mat point_image;
  perspectiveTransform(point_world, point_image, world2cam_);
  if (point_world.rows != 3 || point_world.cols != 1) {
    ROS_WARN("Point transformed to image dimensions has invalid dimensions");
    return false;
  }
  res.image_point.x = point_image.at<double>(0,0);
  res.image_point.y = point_image.at<double>(1,0);
  res.image_point.z = 0.0;
  // todo: move to sensor_msgs::CameraInfo for transformation, uses tf as base, could use as unified interface
  return true;
}

bool WarpContent::imageToWorld(drive_ros_image_recognition::ImageToWorld::Request  &req,
                               drive_ros_image_recognition::ImageToWorld::Response &res)
{
  ROS_DEBUG("WorldToImage service: transforming incoming image coordinates to world coordinates");
  cv::Mat point_world = cv::Mat::zeros(3,1,CV_64F);
  point_world.at<double>(0,0) = req.image_point.x;
  point_world.at<double>(1,0) = req.image_point.y;

  cv::Mat point_image;
  perspectiveTransform(point_world, point_image, cam2world_);
  if (point_world.rows != 3 || point_world.cols != 1) {
    ROS_WARN("Point transformed to world dimensions has invalid dimensions");
    return false;
  }
  res.world_point.x = point_image.at<double>(0,0);
  res.world_point.y = point_image.at<double>(1,0);
  res.world_point.z = point_image.at<double>(2,0);;
  // todo: move to sensor_msgs::CameraInfo for transformation, uses tf as base, could use as unified interface
  return true;
}

} // namespace drive_ros_image_recognition

