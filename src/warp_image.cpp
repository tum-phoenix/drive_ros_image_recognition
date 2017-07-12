#include <drive_ros_image_recognition/warp_image.h>
#include <pluginlib/class_list_macros.h>

namespace drive_ros_image_recognition {

WarpContent::WarpContent(const ros::NodeHandle& nh, const ros::NodeHandle& pnh):
  nh_(nh),
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
  cam_model_()
{
  img_pub_ = it_.advertise("warped_out", 1);
  undistort_pub_ = it_.advertise("undistort_out", 1);
}

WarpContent::~WarpContent() {
}

bool WarpContent::init() {
  if (!getHomographyMatParam(pnh_, world2cam_, "world2cam"))
      return false;

  if (!getHomographyMatParam(pnh_, cam2world_, "cam2world"))
      return false;

  // initialize combined subscriber for camera image and model
  cam_sub_ = it_.subscribeCamera("img_in", 10, &WarpContent::world_image_callback, this);
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
  cv::undistort(current_image_, undistorted_mat, cam_model_.fullIntrinsicMatrix(), cam_model_.distortionCoeffs());
  undistort_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_8UC1, undistorted_mat).toImageMsg());
  // opionally: apply scaled homography
  //  cv::warpPerspective(current_image_, current_image_, S*world2cam_*S.inv(), current_image_.size(),cv::WARP_INVERSE_MAP);

  // flag ensures that we directly use the matrix, as it is done in LMS
//  cv::warpPerspective(undistorted_mat, undistorted_mat, world2cam_, current_image_.size(),cv::WARP_INVERSE_MAP);
//  img_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_8UC1, undistorted_mat).toImageMsg());
}

void WarpImageNodelet::onInit()
{
  my_content_.reset(new WarpContent(getNodeHandle(),ros::NodeHandle(getPrivateNodeHandle())));
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
