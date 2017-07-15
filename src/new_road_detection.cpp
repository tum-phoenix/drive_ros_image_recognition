#include "drive_ros_image_recognition/new_road_detection.h"
#include <pluginlib/class_list_macros.h>

namespace drive_ros_image_recognition {

NewRoadDetection::NewRoadDetection(const ros::NodeHandle nh, const ros::NodeHandle pnh):
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
  line_output_pub_(),
  it_(pnh),
#ifdef PUBLISH_DEBUG
  debug_img_pub_(),
  detected_points_pub_(),
  filtered_points_pub_(),
#endif
  dsrv_server_(),
  dsrv_cb_(boost::bind(&NewRoadDetection::reconfigureCB, this, _1, _2)),
  lines_(),
  linesToProcess_(0),
  current_image_(),
  road_points_buffer_(),
  transform_helper_(),
  image_operator_(),
  img_sub_debug_()
{
}

bool NewRoadDetection::init() {
  // load parameters
  if (!pnh_.getParam("new_road_detection/searchOffset", searchOffset_)) {
    ROS_WARN_STREAM("Unable to load 'searchOffset' parameter, using default: "<<searchOffset_);
  }

  if (!pnh_.getParam("new_road_detection/distanceBetweenSearchlines", distanceBetweenSearchlines_)) {
    ROS_WARN_STREAM("Unable to load 'distanceBetweenSearchlines' parameter, using default: "<<distanceBetweenSearchlines_);
  }

  if (!pnh_.getParam("new_road_detection/minLineWidthMul", minLineWidthMul_)) {
    ROS_WARN_STREAM("Unable to load 'minLineWidthMul' parameter, using default: "<<minLineWidthMul_);
  }

  if (!pnh_.getParam("new_road_detection/maxLineWidthMul", maxLineWidthMul_)) {
    ROS_WARN_STREAM("Unable to load 'maxLineWidthMul' parameter, using default: "<<maxLineWidthMul_);
  }

  if (!pnh_.getParam("new_road_detection/findBySobel", findPointsBySobel_)) {
    ROS_WARN_STREAM("Unable to load 'findBySobel' parameter, using default: "<<findPointsBySobel_);
  }

  if (!pnh_.getParam("new_road_detection/brightness_threshold", brightness_threshold_)) {
    ROS_WARN_STREAM("Unable to load 'threshold' parameter, using default: "<<brightness_threshold_);
  }

  if (!pnh_.getParam("new_road_detection/sobelThreshold", sobelThreshold_)) {
    ROS_WARN_STREAM("Unable to load 'sobelThreshold' parameter, using default: "<<sobelThreshold_);
  }

  if (!pnh_.getParam("new_road_detection/laneWidthOffsetInMeter", laneWidthOffsetInMeter_)) {
    ROS_WARN_STREAM("Unable to load 'laneWidthOffsetInMeter' parameter, using default: "<<laneWidthOffsetInMeter_);
  }

  if (!pnh_.getParam("new_road_detection/translateEnvironment", translateEnvironment_)) {
    ROS_WARN_STREAM("Unable to load 'translateEnvironment' parameter, using default: "<<translateEnvironment_);
  }

  if (!pnh_.getParam("new_road_detection/useWeights", useWeights_)) {
    ROS_WARN_STREAM("Unable to load 'useWeights' parameter, using default: "<<useWeights_);
  }

  transform_helper_.init(pnh_);
  image_operator_ = ImageOperator(transform_helper_);

  dsrv_server_.setCallback(dsrv_cb_);

  // to synchronize incoming images and the road, we use message filters
//  img_sub_.reset(new image_transport::SubscriberFilter(it_,"img_in", 1000));
//  road_sub_.reset(new message_filters::Subscriber<RoadLane>(pnh_,"road_in", 1000));
//  sync_.reset(new message_filters::Synchronizer<SyncImageToRoad>(SyncImageToRoad(100), *img_sub_, *road_sub_));
//  sync_->setAgePenalty(1.0);
//  sync_->registerCallback(boost::bind(&NewRoadDetection::syncCallback, this, _1, _2));

  img_sub_debug_ = it_.subscribe("img_in", 1000, &NewRoadDetection::debugImageCallback, this);

  line_output_pub_ = pnh_.advertise<drive_ros_image_recognition::RoadLane>("line_out",10);

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
  return true;
}

void NewRoadDetection::debugImageCallback(const sensor_msgs::ImageConstPtr& img_in) {
  current_image_ = convertImageMessage(img_in);
  // crop image to 410 width
  cv::Rect roi(cv::Point(current_image_->cols/2-(410/2),0),cv::Point(current_image_->cols/2+(410/2),current_image_->rows));
  search_direction search_dir = search_direction::x;
  search_method search_meth = search_method::sobel;
  search_method search_meth_brightness = search_method::brightness;
  image_operator_.debugPointsImage((*current_image_)(roi), search_dir, search_meth);
  image_operator_.debugPointsImage((*current_image_)(roi), search_dir, search_meth_brightness);
}

void NewRoadDetection::syncCallback(const sensor_msgs::ImageConstPtr& img_in, const RoadLaneConstPtr& road_in){
  current_image_ = convertImageMessage(img_in);
  road_points_buffer_ = road_in->points;

  //if we have a road(?), try to find the line
  if(road_in->points.size() > 1){
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

bool NewRoadDetection::find(){
  //clear old lines
  ROS_INFO("Creating new lines");
  lines_.clear();
  linesToProcess_ = 0;

  //create left/mid/right lane
  linestring mid, left, right;
  for (auto point : road_points_buffer_) {
    boost::geometry::append(mid,point_xy(point.x, point.y));
  }
  // simplify mid (previously done with distance-based algorithm
  // skip this for now, can completely clear the line
//  drive_ros_geometry_common::simplify(mid, mid, distanceBetweenSearchlines_);
  drive_ros_geometry_common::moveOrthogonal(mid, left, -0.4);
  drive_ros_geometry_common::moveOrthogonal(mid, right, 0.4);
  if(mid.size() != left.size() || mid.size() != right.size()){
    ROS_ERROR("Generated lane sizes do not match! Aborting search!");
    return false;
  }
  //get all lines
  auto it_mid = mid.begin();
  auto it_left = left.begin();
  auto it_right = right.begin();
  for(int it = 0; it<mid.size(); it++, ++it_mid, ++it_right, ++it_left){
    SearchLine l;
    // todo: probably wrong, check which exactly to get, will need end and start on the first probably
    l.wStart = cv::Point2f(it_left->x(),it_left->y());
    l.wEnd = cv::Point2f(it_right->x(),it_right->y());
    l.wLeft = cv::Point2f(it_left->x(),it_left->y());
    l.wMid = cv::Point2f(it_mid->x(),it_mid->y());
    l.wRight = cv::Point2f(it_right->x(),it_right->y());
    //check if the part is valid (middle should be always visible)

    cv::Point l_w_start, l_w_end;
    transform_helper_.worldToImage(l.wStart, l.iStart);
    transform_helper_.worldToImage(l.wEnd, l.iEnd);
    cv::Rect img_rect(cv::Point(),cv::Point(current_image_->cols,current_image_->rows));
    // if the left side is out of the current view, use middle instead
    if(!img_rect.contains(l_w_start)){
      // try middle lane -> should always be in image
      l.wStart = cv::Point2f(it_mid->x(),it_mid->y());
      transform_helper_.worldToImage(l.wMid, l.iStart);
      if(img_rect.contains(l_w_start)){
        continue;
      }
    // if the right side is out of the current view, use middle instead
    }else if(!img_rect.contains(l_w_end)){
      l.wEnd = cv::Point2f(it_mid->x(),it_mid->y());
    }

    transform_helper_.worldToImage(l.wStart,l.iStart);
    transform_helper_.worldToImage(l.wEnd,l.iEnd);
    lines_.push_back(l);
    for(SearchLine &l:lines_){
      processSearchLine(l);
    }
  }
  return true;
}

void NewRoadDetection::processSearchLine(const SearchLine &l) {
//  std::vector<int> xv;
//  std::vector<int> yv;

  //calculate the offset
  // done in search function
//  float iDist = cv::norm(l.iEnd - l.iStart);
//  float wDist = cv::norm(l.wEnd - l.wStart);
  // add search offset -> do not need this as we do it directly in the image
//  float pxlPerDist = iDist/wDist*searchOffset_;
//  cv::Point2f iDiff = (l.i_start-l.i_end)/norm(l.i_start-l.i_end);
//  cv::Point2f startLine = cv::Point2f(l.i_start)+iDiff*pxlPerDist;
//  cv::Point2f endLine = cv::Point2f(l.i_end)-iDiff*pxlPerDist;
  //    //get all points in between
  //    lms::math::bresenhamLine(startLine.x,startLine.y,endLine.x,endLine.y,xv,yv); //wir suchen von links nach rechts!

  std::vector<cv::Point2f> foundPoints;

  //find points
  if(findPointsBySobel_){
    // todo: investigate why line width was hard-coded at 0.02 -> maybe add parameter
    foundPoints = image_operator_.returnValidPoints(l, *current_image_, 0.02, search_direction::x, search_method::sobel);
  }else{
    // todo: investigate why line width was hard-coded at 0.02 -> maybe add parameter
    foundPoints = image_operator_.returnValidPoints(l, *current_image_, 0.02, search_direction::x, search_method::brightness);
  }

  // draw unfiltered image points
#if defined(DRAW_DEBUG) || defined(PUBLISH_DEBUG)
    cv::namedWindow("Unfiltered points", CV_WINDOW_NORMAL);
    cv::Mat found_points_mat = current_image_->clone();
    for (auto point : foundPoints) {
      cv::circle(found_points_mat, point, 2, cv::Scalar(255));
    }
    // draw search line
    cv::line(found_points_mat, l.iStart, l.iEnd, cv::Scalar(255));
#ifdef DRAW_DEBUG
    cv::imshow("Unfiltered points", found_points_mat);
    cv::waitKey(1);
#endif

#ifdef PUBLISH_DEBUG
    detected_points_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_8UC1, found_points_mat).toImageMsg());
#endif
#endif

  //filter
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
        if((distance > 0.4-laneWidthOffsetInMeter_ && distance < 0.4 + laneWidthOffsetInMeter_)|| (distance > 0.8-laneWidthOffsetInMeter_ && distance < 0.8+laneWidthOffsetInMeter_)){
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
      transform_helper_.worldToImage(point, img_point);
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
  for(int i = 0; i < (int)validPoints.size(); i++){
    float distanceToLeft = cv::norm(validPoints[i]-l.wLeft);
    float distanceToRight= cv::norm(validPoints[i]-l.wRight);
    float distanceToMid= cv::norm(validPoints[i]-l.wMid);
    // normalize distance
    diff = (l.wLeft - l.wRight)/cv::norm(l.wLeft-l.wRight);
    if(distanceToLeft < distanceToMid && distanceToLeft < distanceToRight){
      //left point
      validPoints[i]-=diff*0.4;
      distances.push_back(distanceToLeft);
    }else if(distanceToRight < distanceToMid ){
      validPoints[i]+=diff*0.4;
      distances.push_back(distanceToRight);
    }else{
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
  drive_ros_image_recognition::RoadLane lane_out;
  lane_out.header.stamp = ros::Time::now();
  geometry_msgs::Point32 point_temp;
  point_temp.z = 0.f;
  for (auto point: validPoints) {
    point_temp.x = point.x;
    point_temp.y = point.y;
    lane_out.points.push_back(point_temp);
  }
  lane_out.roadStateType = drive_ros_image_recognition::RoadLane::UNKNOWN;
  line_output_pub_.publish(lane_out);
}

void NewRoadDetection::reconfigureCB(drive_ros_image_recognition::LineDetectionConfig &config, uint32_t level){
  image_operator_.setConfig(config);
  searchOffset_ = config.searchOffset;
  findPointsBySobel_ = config.findBySobel;
  minLineWidthMul_ = config.minLineWidthMul;
  maxLineWidthMul_ = config.maxLineWidthMul;
  brightness_threshold_ = config.brightness_threshold;
  laneWidthOffsetInMeter_ = config.laneWidthOffsetInMeter;
  useWeights_ = config.useWeights;
  sobelThreshold_ = config.sobelThreshold;
}

NewRoadDetection::~NewRoadDetection() {
}

void NewRoadDetectionNodelet::onInit() {
  new_road_detection_.reset(new NewRoadDetection(getNodeHandle(),getPrivateNodeHandle()));
  if (!new_road_detection_->init()) {
    ROS_ERROR("New_road_detection nodelet failed to initialize");
    // nodelet failing will kill the entire loader anyway
    ros::shutdown();
  }
  else {
    ROS_INFO("New road detection nodelet succesfully initialized");
  }
}

} // namespace drive_ros_image_recognition

PLUGINLIB_EXPORT_CLASS(drive_ros_image_recognition::NewRoadDetectionNodelet, nodelet::Nodelet)
