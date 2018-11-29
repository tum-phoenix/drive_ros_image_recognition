#include <drive_ros_image_recognition/warp_image.h>
#include <pluginlib/class_list_macros.h>

namespace drive_ros_image_recognition {

WarpContent::WarpContent(const ros::NodeHandle& nh, const ros::NodeHandle& pnh):
  nh_(nh),
  pnh_(pnh),
  current_image_(),
  cam_mat_(3,3,CV_64F,cv::Scalar(0.0)),
  dist_coeffs_(8,1,CV_64F,cv::Scalar(0.0)),
  it_(nh),
  cam_sub_(),
  worldToImageServer_(),
  imageToWorldServer_(),
  cam_model_(),
  homography_params_sub_(),
  cam2world_(3,3,CV_64FC1,cv::Scalar(0.0)),
  world2cam_(3,3,CV_64FC1,cv::Scalar(0.0)),
  scaling_mat_init_(3,3,CV_64FC1,cv::Scalar(0.0)),
  scaling_mat_(3,3,CV_64FC1,cv::Scalar(0.0)),
  scaling_mat_inv_(3,3,CV_64FC1,cv::Scalar(0.0)),
  transformed_size_(0,0),
  homo_received_(false),
  output_image_type_(CV_8UC1)
{
  img_pub_ = it_.advertise("warped_out", 1);
  undistort_pub_ = it_.advertise("undistort_out", 1);
}

WarpContent::~WarpContent() {
}

bool WarpContent::init() {
  std::vector<double> world_size;
  double test;
  ROS_INFO_STREAM("PNH namespace: "<<pnh_.getNamespace());
  ROS_INFO_STREAM("Parameter exists: "<<pnh_.hasParam("world_size"));

  if(!pnh_.getParam("world_size", world_size)) {
     ROS_ERROR("Unable to load parameter world_size!");
     return false;
  }
  ROS_ASSERT(world_size.size() == 2);
  std::vector<double> image_size;
  if(!pnh_.getParam("image_size", image_size)) {
    ROS_ERROR("Unable to load parameter image_size!");
    return false;
  }
  if(!pnh_.getParam("output_image_type", output_image_type_))
    ROS_INFO_STREAM("Unable to load parameter output_image_type, using default "<<output_image_type_);

  ROS_ASSERT(image_size.size() == 2);
  transformed_size_ = cv::Size(image_size[0],image_size[1]);
  scaling_mat_init_.at<double>(0,0) = world_size[0]/image_size[0];
  scaling_mat_init_.at<double>(1,1) = -world_size[1]/image_size[1];
  scaling_mat_init_.at<double>(1,2) = world_size[1]/2;
  scaling_mat_init_.at<double>(2,2) = 1.0;
  ROS_INFO_STREAM("Calculated world2mat scaling matrix: "<<scaling_mat_);

  homography_params_sub_ = nh_.subscribe<drive_ros_msgs::Homography>("homography_in", 1,
                                          boost::bind(homography_callback, _1,
                                                      std::ref(cam2world_), std::ref(world2cam_),
                                                      std::ref(scaling_mat_),
                                                      std::ref(scaling_mat_inv_), std::ref(homo_received_),
                                                      std::ref(scaling_mat_init_)));
  // initialize combined subscriber for camera image and model
  cam_sub_ = it_.subscribeCamera("img_in", 10, &WarpContent::world_image_callback, this);
  return true;
}

void WarpContent::world_image_callback(const sensor_msgs::ImageConstPtr& msg,
                                       const sensor_msgs::CameraInfoConstPtr& info_msg) {
  if (!homo_received_) {
    ROS_WARN_THROTTLE(10, "Homography callback not yet received, waiting");
    return;
  }

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

  // undistort and apply homography transformation
  cv::Mat undistorted_mat;
  cv::undistort(current_image_, undistorted_mat, cam_model_.fullIntrinsicMatrix(), cam_model_.distortionCoeffs());
  if (output_image_type_ == CV_8UC3)
    cv::cvtColor(undistorted_mat, undistorted_mat, CV_GRAY2BGR);
  else
    ROS_DEBUG_STREAM("Unsupported output image type "<<output_image_type_<<" using default: "<<CV_8UC1<<" instead");
  std::string output_encoding_str = sensor_msgs::image_encodings::TYPE_8UC1;
  if (output_image_type_ == CV_8UC3)
    output_encoding_str = sensor_msgs::image_encodings::BGR8;

  undistort_pub_.publish(cv_bridge::CvImage(msg->header, output_encoding_str,
                                            undistorted_mat).toImageMsg());

//  cv::Mat topView2cam = world2cam_ * pixels_to_meters;

  // flag ensures that we directly use the matrix, as it is done in LMS
  cv::Mat output_mat;
  cv::warpPerspective(undistorted_mat, output_mat, scaling_mat_, transformed_size_, cv::WARP_INVERSE_MAP);
#ifdef DRAW_DEBUG
  cv::namedWindow("Homographied",CV_WINDOW_NORMAL);
  cv::imshow("Homographied",output_mat);
  cv::waitKey(1);
#endif

  img_pub_.publish(cv_bridge::CvImage(msg->header, output_encoding_str, output_mat).toImageMsg());
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
