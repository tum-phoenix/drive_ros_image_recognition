#include "drive_ros_image_recognition/road_detection.h"
#include <pluginlib/class_list_macros.h>

namespace drive_ros_image_recognition {

RoadDetection::RoadDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh):
  nh_(nh),
  pnh_(pnh),
//  img_sub_(),
//  road_sub_(),
//  sync_(),
  searchOffset_(0.15),
  distanceBetweenSearchlines_(0.1),
  minLineWidthMul_(0.5),
  maxLineWidthMul_(2),
  findPointsBySobel_(true),
  brightness_threshold_(50),
  sobelThreshold_(50),
  laneWidthOffsetInMeter_(0.1),
  translateEnvironment_(false),
  useWeights_(false),
  laneWidth_(0.4),
  line_output_pub_(),
  it_(nh),
#ifdef PUBLISH_DEBUG
  debug_img_pub_(),
  detected_points_pub_(),
  filtered_points_pub_(),
#endif
  dsrv_server_(),
  dsrv_cb_(boost::bind(&RoadDetection::reconfigureCB, this, _1, _2)),
  lines_(),
  linesToProcess_(0),
  current_image_(),
  road_points_buffer_(),
  image_operator_()
,  img_sub_debug_()
, mutex_()
, last_received_transform_()
, tf_listener_()
, road_hints_buffer_()
#ifdef PUBLISH_WORLD_POINTS
  ,world_point_pub_()
#endif
{
}

bool RoadDetection::init() {
  // load parameters
  if (!pnh_.getParam("road_detection/searchOffset", searchOffset_)) {
    ROS_WARN_STREAM("Unable to load 'searchOffset' parameter, using default: "<<searchOffset_);
  }

  if (!pnh_.getParam("road_detection/distanceBetweenSearchlines", distanceBetweenSearchlines_)) {
    ROS_WARN_STREAM("Unable to load 'distanceBetweenSearchlines' parameter, using default: "<<distanceBetweenSearchlines_);
  }

  if (!pnh_.getParam("road_detection/minLineWidthMul", minLineWidthMul_)) {
    ROS_WARN_STREAM("Unable to load 'minLineWidthMul' parameter, using default: "<<minLineWidthMul_);
  }

  if (!pnh_.getParam("road_detection/maxLineWidthMul", maxLineWidthMul_)) {
    ROS_WARN_STREAM("Unable to load 'maxLineWidthMul' parameter, using default: "<<maxLineWidthMul_);
  }

  if (!pnh_.getParam("road_detection/findBySobel", findPointsBySobel_)) {
    ROS_WARN_STREAM("Unable to load 'findBySobel' parameter, using default: "<<findPointsBySobel_);
  }

  if (!pnh_.getParam("road_detection/brightness_threshold", brightness_threshold_)) {
    ROS_WARN_STREAM("Unable to load 'threshold' parameter, using default: "<<brightness_threshold_);
  }

  if (!pnh_.getParam("road_detection/sobelThreshold", sobelThreshold_)) {
    ROS_WARN_STREAM("Unable to load 'sobelThreshold' parameter, using default: "<<sobelThreshold_);
  }

  if (!pnh_.getParam("road_detection/laneWidthOffsetInMeter", laneWidthOffsetInMeter_)) {
    ROS_WARN_STREAM("Unable to load 'laneWidthOffsetInMeter' parameter, using default: "<<laneWidthOffsetInMeter_);
  }

  if (!pnh_.getParam("road_detection/translateEnvironment", translateEnvironment_)) {
    ROS_WARN_STREAM("Unable to load 'translateEnvironment' parameter, using default: "<<translateEnvironment_);
  }

  if (!pnh_.getParam("road_detection/useWeights", useWeights_)) {
    ROS_WARN_STREAM("Unable to load 'useWeights' parameter, using default: "<<useWeights_);
  }

  image_operator_ = ImageOperator();
  if (!image_operator_.init()) {
    ROS_ERROR("Failed to initialize ImageOperator, shutting down!");
    return false;
  }

  std::string world_frame("/rear_axis_middle");
  if (!pnh_.getParam("world_frame", world_frame)) {
    ROS_WARN_STREAM("Unable to load 'useWeights' parameter, using default: "<<world_frame);
  }
  image_operator_.setWorldFrame(world_frame);

  // temporary solution until we set the correct frame
  std::string camera_frame("/camera_optical");
  if (!pnh_.getParam("camera_frame", camera_frame)) {
    ROS_WARN_STREAM("Unable to load 'camera_frame' parameter, using default: "<<camera_frame);
  }
  image_operator_.setCameraFrame(camera_frame);

  dsrv_server_.setCallback(dsrv_cb_);

#ifdef PUBLISH_WORLD_POINTS
  world_point_pub_ = pnh_.advertise<geometry_msgs::PointStamped>("worldPoints", 1000);
#endif

  // fill initial hints -> straight line with 10cm spacing
  geometry_msgs::PointStamped temp_point;
  temp_point.header.frame_id = world_frame;
  temp_point.point.x = 0;
  temp_point.point.y = 0;
  temp_point.point.z = 0;
  for (int i = 3; i < 7; ++i) {
    temp_point.point.x = i*0.1;
    road_points_buffer_.push_back(temp_point);
  }

  img_sub_ =  it_.subscribe("img_in", 1000, &RoadDetection::imageCallback, this);

  // to synchronize incoming images and the road, we use message filters
//  img_sub_.reset(new image_transport::SubscriberFilter(it_,"img_in", 10));
//  img_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_,"/warped_image",5));

////  message_filters::Subscriber<sensor_msgs::Image> test_sub(pnh_,"img_in", 1000);
//  road_sub_.reset(new message_filters::Subscriber<drive_ros_msgs::RoadLane>(pnh_,"/road_detection/road_in", 5));
////  sync_->registerCallback(boost::bind(&RoadDetection::syncCallback, this, _1, _2));

//  sync_.reset(new message_filters::Synchronizer<SyncImageToRoad>(SyncImageToRoad(5), *img_sub_, *road_sub_));
////  sync_.reset(new message_filters::Synchronizer<SyncImageToRoad>(SyncImageToRoad(1), *img_sub_, test_sub));
////  sync_->setAgePenalty(1.0);
//  ROS_INFO("BEFORE REGISTERING CALLBACK");
//  sync_->registerCallback(boost::bind(&RoadDetection::syncCallback, this, _1, _2));

//  // Debug callbacks for testing
////  img_sub_debug_ = it_.subscribe("img_in", 1000, &RoadDetection::debugImageCallback, this);
////  img_sub_debug_ = it_.subscribe("img_in", 1000, boost::bind(&RoadDetection::debugDrawFrameCallback, this,
////                                                             _1, std::string("/camera_optical"), std::string("/front_axis_middle")));

  line_output_pub_ = nh_.advertise<drive_ros_msgs::RoadLane>("line_out",10);

#ifdef PUBLISH_DEBUG
  debug_img_pub_ = it_.advertise("debug_image_out", 10);
  detected_points_pub_ = it_.advertise("detected_points_out", 10);
  filtered_points_pub_ = it_.advertise("filtered_points_out", 10);
#endif

  // todo: we have not decided on an interface for these debug channels yet
  //    debugAllPoints = writeChannel<lms::math::polyLine2f>("DEBUG_ALL_POINTS");
  //    debugValidPoints = writeChannel<lms::math::polyLine2f>("DEBUG_VALID_POINTS");
  //    debugTranslatedPoints = writeChannel<lms::math::polyLine2f>("DEBUG_VALID_TRANSLATED_POINTS");

  // todo: we have not defined the interface for this yet
  //    car = readChannel<street_environment::CarCommand>("CAR"); //TODO create ego-estimation service

  try {
    ros::Duration timeout(1);
    tf_listener_.waitForTransform("/odom", world_frame,
                                  ros::Time::now(), timeout);
    tf_listener_.lookupTransform("/odom", world_frame,
                                 ros::Time::now(), last_received_transform_);
  }
  catch (tf::TransformException& ex) {
    ROS_WARN("[debugDrawFrameCallback] TF exception:\n%s", ex.what());
    return false;
  }

  return true;
}

void RoadDetection::debugImageCallback(const sensor_msgs::ImageConstPtr& img_in) {
  current_image_ = convertImageMessage(img_in);
  // crop image to 410 width
  cv::Rect roi(cv::Point(current_image_->cols/2-(410/2),0),cv::Point(current_image_->cols/2+(410/2),current_image_->rows));
  search_direction search_dir = search_direction::x;
  search_method search_meth = search_method::sobel;
  search_method search_meth_brightness = search_method::brightness;
  image_operator_.debugPointsImage((*current_image_)(roi), search_dir, search_meth);
  image_operator_.debugPointsImage((*current_image_)(roi), search_dir, search_meth_brightness);
}

// draws draw_frame in camera image (if visible)
void RoadDetection::debugDrawFrameCallback(const sensor_msgs::ImageConstPtr& img_in,
                                           const std::string camera_frame,
                                           const std::string draw_frame) {
  current_image_ = convertImageMessage(img_in);

  tf::TransformListener tf_listener_;
  tf::StampedTransform transform;
  try {
    ros::Duration timeout(1);
    tf_listener_.waitForTransform(camera_frame, draw_frame,
                                  ros::Time::now(), timeout);
    tf_listener_.lookupTransform(camera_frame, draw_frame,
                                 ros::Time::now(), transform);
  }
  catch (tf::TransformException& ex) {
    ROS_WARN("[debugDrawFrameCallback] TF exception:\n%s", ex.what());
    return;
  }

  tf::Point pt = transform.getOrigin();
  cv::Point3f pt_cv(pt.x(), pt.y(), pt.z());
  cv::Point uv;
  image_operator_.worldToImage(pt_cv, uv);

  cv::Mat draw_image;
  cv::cvtColor(*current_image_, draw_image, cv::COLOR_GRAY2BGR);
  cv::circle(draw_image, uv, 3, CV_RGB(255,0,0), -1);
  cv::namedWindow("frame in image", CV_WINDOW_NORMAL);
  cv::imshow("frame in image", draw_image);
  cv::waitKey(1);
}

void RoadDetection::imageCallback(const sensor_msgs::ImageConstPtr& img_in) {
  // transform hint points to the next moved frame
//  tf::Transform point_transform;
//  tf::Transform temp_transform;
//  tf_listener_.lookupTransform("/odom", image_operator_.getWorldFrame(), road_in->points.front().header.stamp, temp_transform);
//  point_transform = last_received_transform_.inverseTimes(temp_transform);
  try
  {
    current_image_ = convertImageMessage(img_in);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // maybe tf is smart enough to use based on the stamps and do this
  std::vector<geometry_msgs::PointStamped> moved_points;
  geometry_msgs::PointStamped temp_point;
  for (const geometry_msgs::PointStamped& last_point : road_points_buffer_) {
    tf_listener_.transformPoint(image_operator_.getWorldFrame(), last_point, temp_point);
    moved_points.push_back(temp_point);
  }

  int rect_offset = 64;
  image_operator_.setImageRect(cv::Rect(0, rect_offset, current_image_->cols, current_image_->rows-rect_offset));

  std::lock_guard<std::mutex> guard(mutex_);

  // since we are handling hints internally we cannot have no hints at all
  find();

  if (road_points_buffer_.size() != road_hints_buffer_.size()) {
    ROS_WARN_STREAM("Sizes of new hint points and the last frame hint buffer do not match, using last hints unchanged");
    return;
  }

  // will skip steps in which it failed to find a valid lane midpoint (more than 2 points for now)
  for (int i=0; i<road_hints_buffer_.size(); ++i)
    if (road_hints_buffer_[i].point.x != 0.0 || road_hints_buffer_[i].point.z != 0.0)
      road_points_buffer_[i] = road_hints_buffer_[i];

  return;
}

void RoadDetection::syncCallback(const sensor_msgs::ImageConstPtr& img_in, const drive_ros_msgs::RoadLaneConstPtr& road_in){
  current_image_ = convertImageMessage(img_in);
  // set image checkbox in order to verfiy that transformed points are inside the image
  // todo: add parameter for this
  int rect_offset = 64;
  image_operator_.setImageRect(cv::Rect(0, rect_offset, current_image_->cols, current_image_->rows-rect_offset));
  road_points_buffer_ = road_in->points;
  // debug image display
//  cv::namedWindow("cutout window", CV_WINDOW_NORMAL);
//  cv::imshow("cutout window", (*current_image_)(cv::Rect(0, rect_offset, current_image_->cols, current_image_->rows-rect_offset)));
//  cv::waitKey(1);

  std::lock_guard<std::mutex> guard(mutex_);

  //if we have a road(?), try to find the line
  if(road_in->points.size() >= 1){
    find(); //TODO use bool from find
  } else {
    ROS_WARN("Road buffer has no points stored");
  }


//  float dx = 0;
//  float dy = 0;
//  float dPhi = 0;

  //update the course
  // todo: no access to car command interface yet
  //    if(!translate_environment_){
  //        logger.info("translation")<<car->deltaPhi();
  //        dPhi = car->deltaPhi();
  //    }

  // todo: no interface for this yet, create course verification service to verfiy and publish
  //    lms::ServiceHandle<local_course::LocalCourse> localCourse = getService<local_course::LocalCourse>("LOCAL_COURSE_SERVICE");
  //    if(localCourse.isValid()){
  //        logger.time("localCourse");
  //        localCourse->update(dx,dy,dPhi);
  //        *road = localCourse->getCourse();
  //        logger.timeEnd("localCourse");
#ifdef DRAW_DEBUG
  // todo: no interface for this yet
  //        if(renderDebugImage){
  //                debugTranslatedPoints->points() = localCourse->getPointsAdded();
  //        }
#endif
  // should do something like this
  //    line_output_pub_.publish(road_translated);

  //    }else{
  //        ROS_ERROR("LocalCourse invalid, aborting the callback!");
  //        return;
  //    }
}

bool RoadDetection::find(){
  //clear old lines
  ROS_INFO("Creating new lines");
  lines_.clear();
  linesToProcess_ = 0;
  // we want at least two line points, otherwise we will simply not have any lines (duh)
  if (road_points_buffer_.size() < 2) {
    ROS_WARN("Less than 2 points in received RoadLane message - not using it.");
    return false;
  }

  //create left/mid/right lane
  linestring mid, left, right;
  // we actually have to move the points in image frame first, and then transform them -> make more efficient later
  // either move boost to 3d points, or store the z coordinate somehow
  // for now redundant transforms
  for (geometry_msgs::PointStamped& point_stamped : road_points_buffer_) {
    boost::geometry::append(mid,point_xy(point_stamped.point.x, point_stamped.point.y));
  }
  // simplify mid (previously done with distance-based algorithm
  // skip this for now, can completely clear the line
//  drive_ros_geometry_common::simplify(mid, mid, distanceBetweenSearchlines_);
  drive_ros_geometry_common::moveOrthogonal(mid, left, -laneWidth_);
  drive_ros_geometry_common::moveOrthogonal(mid, right, laneWidth_);
  if(mid.size() != left.size() || mid.size() != right.size()){
    ROS_ERROR("Generated lane sizes do not match! Aborting search!");
    return false;
  }

  // make transforms from whatever frame the points are in to the world
  // this is ugly, put it here just to test if it works for now
  geometry_msgs::PointStamped moved_point;
  geometry_msgs::PointStamped moved_point_camera;
  // assuming all points in message are in the same frame (they should)
  moved_point.header.frame_id = road_points_buffer_.front().header.frame_id;
  std::vector<cv::Point3d> mid_temp, left_temp, right_temp;
  auto it_mid_bg = mid.begin();
  auto it_left_bg = left.begin();
  auto it_right_bg = right.begin();
  auto it_original = road_points_buffer_.begin();
  for(int it = 0; it<mid.size(); ++it, ++it_mid_bg, ++it_original, ++it_right_bg, ++it_left_bg){
    moved_point.point.x = it_mid_bg->x();
    moved_point.point.y = it_mid_bg->y();
    moved_point.point.z = it_original->point.z;
    image_operator_.transformPointToImageFrame(moved_point,moved_point_camera);
    mid_temp.push_back(cv::Point3d(moved_point_camera.point.x, moved_point_camera.point.y, moved_point_camera.point.z));
    moved_point.point.x = it_left_bg->x();
    moved_point.point.y = it_left_bg->y();
    moved_point.point.z = it_original->point.z;
    image_operator_.transformPointToImageFrame(moved_point,moved_point_camera);
    left_temp.push_back(cv::Point3d(moved_point_camera.point.x, moved_point_camera.point.y, moved_point_camera.point.z));
    moved_point.point.x = it_right_bg->x();
    moved_point.point.y = it_right_bg->y();
    moved_point.point.z = it_original->point.z;
    image_operator_.transformPointToImageFrame(moved_point,moved_point_camera);
    right_temp.push_back(cv::Point3d(moved_point_camera.point.x, moved_point_camera.point.y, moved_point_camera.point.z));
  }

  //get all lines
  // todo: assert equal length of right, middle and left points (in world frame plane and image)
  bool moved_to_mid = false;
  for(int it = 0; it<mid.size(); it++){
    SearchLine l;
    moved_to_mid = false;
    // those points are in the world frame plane
    l.wStart = cv::Point2f(left[it].x(), left[it].y());
    l.wEnd = cv::Point2f(right[it].x(), right[it].y());
    l.wLeft = cv::Point2f(left[it].x(), left[it].y());
    l.wRight = cv::Point2f(right[it].x(), right[it].y());
    l.wMid = cv::Point2f(mid[it].x(), mid[it].y());
    // for transformation we use points in the optical camera tf frame
    if (!image_operator_.worldToImage(left_temp[it], l.iStart)) {
      ROS_WARN_STREAM("Unable to transform left line start point "<<left_temp[it]<<" to image coordinates, trying middle point");
      // try middle lane -> should always be in image
      moved_to_mid = true;
      // once again-> image plane
      l.wStart = cv::Point2f(mid[it].x(), mid[it].y());
      // once again: camera_optical tf frame
      if (!image_operator_.worldToImage(mid_temp[it], l.iStart)) {
        ROS_WARN_STREAM("Unable to transform middle line point "<<mid_temp[it]<<" to image");
        continue;
      }
    }
    // camera_optical tf frame
    if (!image_operator_.worldToImage(right_temp[it], l.iEnd)) {
      ROS_WARN_STREAM("Unable to transform left line end point "<<right_temp[it]<<" to image coordinates, trying middle point");
      // if the right side is out of the current view, use middle instead
      // do not use it if the left point already was moved to the middle as well
      if (moved_to_mid) {
        ROS_WARN_STREAM("Left point has already been moved to middle, skipping line");
        continue;
      } else {
        // world plane
        l.wEnd = cv::Point2f(mid[it].x(), mid[it].y());
        // camera_optical tf frame
        if(!image_operator_.worldToImage(mid_temp[it], l.iEnd)) {
          ROS_WARN_STREAM("Unable to transform middle line point "<<mid_temp[it]<<" to image");
          continue;
        }
      }
    }
    // correctly order line points
//    fixeLineOrdering(l, search_direction::y);
    lines_.push_back(l);
  }

  // process search lines
  for(SearchLine& l:lines_){
    road_hints_buffer_.push_back(processSearchLine(l));
  }
  return true;
}

geometry_msgs::PointStamped RoadDetection::processSearchLine(const SearchLine &l) {
  std::vector<cv::Point2f> foundPoints;

  // draw search line points in image
#ifdef DRAW_DEBUG
  cv::namedWindow("Line search points", CV_WINDOW_NORMAL);
  cv::Mat debug_image_line;
  cv::cvtColor(current_image_->clone(), debug_image_line, cv::COLOR_GRAY2RGB);
  cv::circle(debug_image_line, l.iStart, 3, cv::Scalar(255,0,0));
  cv::circle(debug_image_line, l.iEnd, 3, cv::Scalar(0,0,255));
  cv::line(debug_image_line, l.iStart, l.iEnd, cv::Scalar(0,255,0));
  cv::imshow("Line search points", debug_image_line);
#endif

  //find points
  if(findPointsBySobel_){
    // todo: investigate why line width was hard-coded at 0.02 -> maybe add parameter
    foundPoints = image_operator_.returnValidPoints(l, *current_image_, 0.02, search_direction::y, search_method::sobel);
  }else{
    // todo: investigate why line width was hard-coded at 0.02 -> maybe add parameter
    // todo: generate kernel based on line angle instead of choosing sobel direction here
    foundPoints = image_operator_.returnValidPoints(l, *current_image_, 0.02, search_direction::y, search_method::brightness);
  }

  if (foundPoints.size() != 0)
    ROS_INFO_STREAM("Found "<<foundPoints.size()<<" points in image!");

  cv::waitKey(1);

  //filter
  // basically checks that points belong to the lanes themselves from the looks
  //TODO filter points that are in poluted regions (for example the car itself)
  //remove points with bad distances
  std::vector<cv::Point2f> validPoints;
  if(foundPoints.size() > 2){
    std::vector<int> foundCounter;
    foundCounter.resize(foundPoints.size(),0);
    for(std::size_t fp = 0; fp < foundPoints.size(); fp++){
      // todo: insert service here
      const cv::Point2f &s = foundPoints[fp];
      for(std::size_t test = fp+1; test <foundPoints.size(); test++){
        // todo: insert service here
        const cv::Point2f &toTest  = foundPoints[test];
        float distance = cv::norm(s-toTest);
        if((distance > laneWidth_-laneWidthOffsetInMeter_ && distance < laneWidth_+laneWidthOffsetInMeter_)|| (distance > 0.8-laneWidthOffsetInMeter_ && distance < 0.8+laneWidthOffsetInMeter_)){
          foundCounter[test]++;
          foundCounter[fp]++;
        }
      }
    }
    //filter, all valid points should have the same counter and have the highest number
    int max = 0;
    for(int c:foundCounter){
      if(c > max){
        max = c;
      }
    }
    for(int i = 0; i < (int)foundPoints.size(); i++){
      if(foundCounter[i] >= max){
        validPoints.push_back(foundPoints[i]);
      }
    }
    //TODO if we have more than 3 points we know that there is an error!
    //        foundPoints = validPoints;
  }
  else {
    // just use the 2 points we found otherwise
    validPoints = foundPoints;
  }

  // draw filtered points
#if defined(DRAW_DEBUG) || defined(PUBLISH_DEBUG)
    // todo: make some more efficient storage method here, will translate back and forth here for now
    cv::namedWindow("Filtered Points", CV_WINDOW_NORMAL);
    cv::Mat filtered_points_mat = current_image_->clone();
    cv::Point img_point;
    for (auto point : validPoints) {
      image_operator_.worldToImage(point, img_point);
      cv::circle(filtered_points_mat, img_point, 2, cv::Scalar(255));
    }
#ifdef DRAW_DEBUG
    cv::imshow("Filtered Points", filtered_points_mat);
#endif
#ifdef PUBLISH_DEBUG
    filtered_points_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_8UC1, filtered_points_mat).toImageMsg());
#endif
#endif

  //Handle found points
  cv::Point2f diff;
  std::vector<float> distances;
  // assign lanes
  // todo: investigate if this can be done more efficiently, f.e. during line detection
  for(int i = 0; i < (int)validPoints.size(); i++){
    float distanceToLeft = cv::norm(validPoints[i]-l.wLeft);
    float distanceToRight= cv::norm(validPoints[i]-l.wRight);
    float distanceToMid= cv::norm(validPoints[i]-l.wMid);
    // normalize distance
    diff = (l.wLeft - l.wRight)/cv::norm(l.wLeft-l.wRight);
    if(distanceToLeft < distanceToMid && distanceToLeft < distanceToRight){
      //left point
      validPoints[i]-=diff*laneWidth_;
      distances.push_back(distanceToLeft);
    }else if(distanceToRight < distanceToMid ){
      // right lane
      validPoints[i]+=diff*laneWidth_;
      distances.push_back(distanceToRight);
    }else{
      // middle lane
      distances.push_back(distanceToMid);
    }
  }

  std::vector<float> weights;
  for(const float &dist:distances){
    //as distance is in meter, we multiply it by 100
    if(useWeights_)
      weights.push_back(1/(dist*100+0.001)); //TODO hier etwas sinnvolles überlegen
    else
      weights.push_back(1);
  }

  // todo: check how this gets handled and adjust the interface
  drive_ros_msgs::RoadLane lane_out;
  lane_out.header.stamp = ros::Time::now();
  geometry_msgs::PointStamped point_temp;
  point_temp.header.frame_id = std::string("rear_axis_middle_ground");
  point_temp.point.z = 0.f;
  for (auto point: validPoints) {
    point_temp.point.x = point.x;
    point_temp.point.y = point.y;
    lane_out.points.push_back(point_temp);
  }
  lane_out.roadStateType = drive_ros_msgs::RoadLane::UNKNOWN;
  line_output_pub_.publish(lane_out);

  geometry_msgs::PointStamped hint_point;
  hint_point.header.frame_id = image_operator_.getWorldFrame();
  hint_point.point.x = 0.0;
  hint_point.point.y = 0.0;
  hint_point.point.z = 0.0;
  // simple method: if we have two points -> average between them
  if (validPoints.size() == 2) {
    cv::Point2f next_hint_point = (validPoints[1] + validPoints[0])/2;
    hint_point.point.x = next_hint_point.x;
    hint_point.point.y = next_hint_point.y;
  }
  return hint_point;
}

void RoadDetection::reconfigureCB(drive_ros_image_recognition::LineDetectionConfig &config, uint32_t level){
  image_operator_.setConfig(config);
  searchOffset_ = config.searchOffset;
  findPointsBySobel_ = config.findBySobel;
  minLineWidthMul_ = config.minLineWidthMul;
  maxLineWidthMul_ = config.maxLineWidthMul;
  brightness_threshold_ = config.brightness_threshold;
  laneWidthOffsetInMeter_ = config.laneWidthOffsetInMeter;
  laneWidth_ = config.lineWidth;
  useWeights_ = config.useWeights;
  sobelThreshold_ = config.sobelThreshold;
}

RoadDetection::~RoadDetection() {
}

void RoadDetectionNodelet::onInit() {
  road_detection_.reset(new RoadDetection(getNodeHandle(),getPrivateNodeHandle()));
  if (!road_detection_->init()) {
    ROS_ERROR("road_detection nodelet failed to initialize");
    // nodelet failing will kill the entire loader anyway
    ros::shutdown();
  }
  else {
    ROS_INFO("road detection nodelet succesfully initialized");
  }
}

} // namespace drive_ros_image_recognition

PLUGINLIB_EXPORT_CLASS(drive_ros_image_recognition::RoadDetectionNodelet, nodelet::Nodelet)
