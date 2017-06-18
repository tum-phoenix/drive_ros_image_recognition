#include <drive_ros_image_recognition/warp_image.h>

namespace drive_ros_image_recognition {

WarpContent::WarpContent(ros::NodeHandle& pnh):
  pnh_(pnh),
  current_image_(),
  world2cam_(3,3,CV_64F,cv::Scalar(0.0)),
  cam2world_(3,3,CV_64F,cv::Scalar(0.0)),
  cam_mat_(3,3,CV_64F,cv::Scalar(0.0)),
  dist_coeffs_(7,1,CV_64F,cv::Scalar(0.0)),
  it_(pnh),
  img_sub_(),
  worldToImageServer_(),
  imageToWorldServer_()
{
  img_pub_ = it_.advertise("warped_out", 1);
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
    world2cam_.at<int>(i) = temp_vals[i];
  }

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
     world2cam_.at<int>(i) = temp_vals[i];
  }

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
  cam_mat_.at<int>(0,0) = temp_vals[0];
  cam_mat_.at<int>(1,1) = temp_vals[1];
  cam_mat_.at<int>(0,2) = temp_vals[2];
  cam_mat_.at<int>(1,2) = temp_vals[3];
  cam_mat_.at<int>(2,2) = 1.0;

  // retreive distortion parameter vector for undistortion
  temp_vals.clear();
  if (!pnh_.getParam("camera_matrix/dist_coeffs", temp_vals)) {
    ROS_ERROR("Unable to load distortion coefficient vector parameters from configuration file!");
    return false;
  }
  if (temp_vals.size() != 5) {
    ROS_ERROR("Retreived distortion coefficient vector does not have 5 values!");
    return false;
  }
  // (row,column) indexing
  dist_coeffs_.at<int>(0,0) = temp_vals[0];
  dist_coeffs_.at<int>(1,0) = temp_vals[1];
  dist_coeffs_.at<int>(4,0) = temp_vals[2];
  dist_coeffs_.at<int>(5,0) = temp_vals[3];
  dist_coeffs_.at<int>(6,0) = temp_vals[4];

  // initialize homography transformation subscriber
  img_sub_ = it_.subscribe("img_in", 10, &WarpContent::world_image_callback, this);
  worldToImageServer_ = pnh_.advertiseService("WorldToImage", &WarpContent::worldToImage, this);
  imageToWorldServer_ = pnh_.advertiseService("ImageToWorld", &WarpContent::imageToWorld, this);
  return true;
}

void WarpContent::world_image_callback(const sensor_msgs::ImageConstPtr& msg) {
  try
  {
    // todo: check if this if we have 8 or 16 greyscale images
    current_image_ = cv_bridge::toCvShare(msg, "mono16")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_WARN("Could not convert incoming image from '%s' to 'CV_16U', skipping.", msg->encoding.c_str());
    return;
  }

  // undistort and apply homography transformation
  cv::undistort(current_image_, current_image_, cam_mat_, dist_coeffs_);
  cv::perspectiveTransform(current_image_, current_image_, world2cam_);
  img_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono16", current_image_).toImageMsg());
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

