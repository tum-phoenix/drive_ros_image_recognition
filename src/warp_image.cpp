#include <drive_ros_image_recognition/warp_image.h>
#include <pluginlib/class_list_macros.h>

namespace drive_ros_image_recognition {

WarpContent::WarpContent(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh)
  , pnh_(pnh)
  , current_image_()
  , it_(nh)
  , cam_sub_()
  , cam_model_()
  , image_operator_()
{
  img_pub_ = it_.advertise("warped_out", 1);
  undistort_pub_ = it_.advertise("undistort_out", 1);
}

WarpContent::~WarpContent() {
}

bool WarpContent::init() {
  // common image operations
  if(!image_operator_.init()) {
    ROS_WARN_STREAM("Failed to init image_operator");
    return false;
  }

  // initialize combined subscriber for camera image and model
  cam_sub_ = it_.subscribeCamera("img_in", 10, &WarpContent::world_image_callback, this);
  return true;
}

void WarpContent::world_image_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
  ROS_INFO("Image warper received image_raw");
  try {
    // copy
    current_image_ = cv_bridge::toCvCopy(msg, "")->image;
  } catch (cv_bridge::Exception& e) {
    ROS_WARN("Could not convert incoming image from '%s' to 'CV_16U', skipping.", msg->encoding.c_str());
    return;
  }

  cam_model_.fromCameraInfo(info_msg);

  // undistort image and publish
  cv::Mat undistorted_mat;
  cv::undistort(current_image_, undistorted_mat, cam_model_.fullIntrinsicMatrix(), cam_model_.distortionCoeffs());
  undistort_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_8UC1, undistorted_mat).toImageMsg());

  // apply homography
  cv::Mat output_mat;
  image_operator_.homographImage(undistorted_mat, output_mat);

#ifdef DRAW_DEBUG
  cv::namedWindow("Homographied",CV_WINDOW_NORMAL);
  cv::imshow("Homographied",output_mat);
  cv::waitKey(1);
#endif
  img_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_8UC1, output_mat).toImageMsg());
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
