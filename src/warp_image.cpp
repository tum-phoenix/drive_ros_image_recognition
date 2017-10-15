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
  homo_received_(false)
{
  img_pub_ = it_.advertise("warped_out", 1);
  undistort_pub_ = it_.advertise("undistort_out", 1);
}

WarpContent::~WarpContent() {
}

bool WarpContent::init() {
  homography_params_sub_ = nh_.subscribe<drive_ros_msgs::Homography>("homography_in", 1,
                                          boost::bind(homography_callback, _1,
                                                      std::ref(cam2world_), std::ref(world2cam_), std::ref(homo_received_)));
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

  // optionally: scale homography, as image is too small
//  cv::Rect roi(cv::Point(current_image_.cols/2-(512/2),0),cv::Point(current_image_.cols/2+(512/2),current_image_.rows));
//  current_image_ = current_image_(roi);
  cv::Mat S = cv::Mat::eye(3,3,CV_64F);
  S.at<double>(0,0) = 0.007;
  S.at<double>(1,1) = 0.007;
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
//  cv::Mat output_mat;
//  // Choose top-view image size
//  cv::Size topViewSize = cv::Size(512, 512);

//  // Choose corner points (in real-world coordinates)
//  std::vector<cv::Point2f> coordinates;
//  coordinates.emplace_back(0, -1.500);
//  coordinates.emplace_back(0, 1.500);
//  coordinates.emplace_back(3.000, -1.500);
//  coordinates.emplace_back(3.000, 1.500);

//  std::vector<cv::Point2f> pixels;
//  pixels.emplace_back(0, topViewSize.height);
//  pixels.emplace_back(0, 0);
//  pixels.emplace_back(topViewSize.width, topViewSize.height);
//  pixels.emplace_back(topViewSize.width, 0);

//  cv::Mat H = cv::findHomography(pixels, coordinates);
//  cv::Mat trafo_mat = world2cam_ * H;

//  cv::Size topViewSize = current_image_.size();

//  std::vector<cv::Point2f> coordinates;
//  coordinates.emplace_back(0, -1.500);
//  coordinates.emplace_back(0, 1.500);
//  coordinates.emplace_back(3.000, -1.500);
//  coordinates.emplace_back(3.000, 1.500);

//  std::vector<cv::Point2f> pixels;
//  pixels.emplace_back(0, topViewSize.height);
//  pixels.emplace_back(0, 0);
//  pixels.emplace_back(topViewSize.width, topViewSize.height);
//  pixels.emplace_back(topViewSize.width, 0);

//  cv::Mat H = cv::findHomography(pixels, coordinates);


//  cv::Mat S = cv::Mat::eye(3,3,CV_64F);
//  S.at<double>(0,0) = 410/current_image_.rows;
//  S.at<double>(1,1) = 752/current_image_.cols;
  cv::Mat output_mat(current_image_.size(),CV_8UC1,cv::Scalar(0));

//  for(int i = 0; i < current_image_.rows; i++)
//  {
//    const double* Mi = current_image_.ptr<double>(i);
//    for(int j = 0; j < current_image_.cols; j++) {
//        float a = i * cam2world_.data[0] + j * cam2world_.data[1] + cam2world_.data[2];
//        float b = i * cam2world_.data[3] + j * cam2world_.data[4] + cam2world_.data[5];
//        float c = i * cam2world_.data[6] + j * cam2world_.data[7] + cam2world_.data[8];
//        int x = a / c;
//        int y = b / c;
////        ROS_INFO_STREAM("x: "<<x<<" y: "<<y);
//        output_mat.at<int>(x,y) = Mi[j];
//      }
//  }

  cv::warpPerspective(undistorted_mat, output_mat, world2cam_*S, current_image_.size(), cv::WARP_INVERSE_MAP);
  cv::namedWindow("Homographied",CV_WINDOW_NORMAL);
  cv::imshow("Homographied",output_mat);
  cv::waitKey(1);
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
