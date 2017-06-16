#include "drive_ros_image_recognition/new_road_detection.h"
//#include <lms/imaging/warp.h>
//#include <lms/math/curve.h>
//#include <lms/imaging/graphics.h>
//#include <local_course/local_course.h>
//#include <street_environment/car.h>
//#include <detection_utils.h>
//#include <warp_service/warp_service.h>

NewRoadDetection::NewRoadDetection(ros::NodeHandle nh, ros::NodeHandle pnh):
  nh_(nh),
  pnh_(pnh),
  img_sub_(),
  my_namespace_("new_road_detection"),
  searchOffset_(0.15),
  distanceBetweenSearchlines_(0.1),
  minLineWidthMul_(0.5),
  maxLineWidthMul_(2),
  findPointsBySobel_(true),
  threshold_(50),
  sobelThreshold_(50),
  laneWidthOffsetInMeter_(0.1),
  translateEnvironment_(false),
  useWeights_(false),
  renderDebugImage_(false),
  numThreads_(4),
  road_sub_(),
  debug_image_(0,0,CV_16UC1),
  line_output_pub_(),
  it_(pnh),
  dsrv_server_(),
  dsrv_cb_(boost::bind(&NewRoadDetection::reconfigureCB, this, _1, _2)),
  lines_(),
  threadsRunning_(false),
  linesToProcess_(0),
  threads_(),
  conditionNewLine_(),
  conditionLineProcessed_()
{
}

bool NewRoadDetection::init() {
  // load parameters
  if (!pnh_.getParam(my_namespace_+"searchOffset", searchOffset_)) {
    ROS_WARN_STREAM("Unable to load 'searchOffset' parameter, using default: "<<searchOffset_);
  }

  if (!pnh_.getParam(my_namespace_+"distanceBetweenSearchlines", distanceBetweenSearchlines_)) {
    ROS_WARN_STREAM("Unable to load 'distanceBetweenSearchlines' parameter, using default: "<<distanceBetweenSearchlines_);
  }

  if (!pnh_.getParam(my_namespace_+"minLineWidthMul", minLineWidthMul_)) {
    ROS_WARN_STREAM("Unable to load 'minLineWidthMul' parameter, using default: "<<minLineWidthMul_);
  }

  if (!pnh_.getParam(my_namespace_+"maxLineWidthMul", maxLineWidthMul_)) {
    ROS_WARN_STREAM("Unable to load 'maxLineWidthMul' parameter, using default: "<<maxLineWidthMul_);
  }

  if (!pnh_.getParam(my_namespace_+"findBySobel", findPointsBySobel_)) {
    ROS_WARN_STREAM("Unable to load 'findBySobel' parameter, using default: "<<findPointsBySobel_);
  }

  if (!pnh_.getParam(my_namespace_+"threshold", threshold_)) {
    ROS_WARN_STREAM("Unable to load 'threshold' parameter, using default: "<<threshold_);
  }

  if (!pnh_.getParam(my_namespace_+"sobelThreshold", sobelThreshold_)) {
    ROS_WARN_STREAM("Unable to load 'sobelThreshold' parameter, using default: "<<sobelThreshold_);
  }

  if (!pnh_.getParam(my_namespace_+"laneWidthOffsetInMeter", laneWidthOffsetInMeter_)) {
    ROS_WARN_STREAM("Unable to load 'laneWidthOffsetInMeter' parameter, using default: "<<laneWidthOffsetInMeter_);
  }

  if (!pnh_.getParam(my_namespace_+"translateEnvironment", translateEnvironment_)) {
    ROS_WARN_STREAM("Unable to load 'translateEnvironment' parameter, using default: "<<translateEnvironment_);
  }

  if (!pnh_.getParam(my_namespace_+"useWeights", useWeights_)) {
    ROS_WARN_STREAM("Unable to load 'useWeights' parameter, using default: "<<useWeights_);
  }

  if (!pnh_.getParam(my_namespace_+"renderDebugImage", renderDebugImage_)) {
    ROS_WARN_STREAM("Unable to load 'renderDebugImage' parameter, using default: "<<renderDebugImage_);
  }

  if (!pnh_.getParam(my_namespace_+"threads", numThreads_)) {
    ROS_WARN_STREAM("Unable to load 'threads' parameter, using default: "<<numThreads_);
  }

  dsrv_server_.setCallback(dsrv_cb_);

  img_sub_ = it_.subscribe("image", 10, &NewRoadDetection::imageCallback, this);
  road_sub_ = nh_.subscribe("road_in", 100, &NewRoadDetection::roadCallback, this);
  line_output_pub_ = nh_.advertise<drive_ros_image_recognition::RoadLaneConstPtr>("line_out",10);

#ifdef DRAW_DEBUG
    debug_img_pub_ = nh_.advertise("/debug_image_out", 10);
#endif

    // todo: we have not decided on an interface for these debug channels yet
//    debugAllPoints = writeChannel<lms::math::polyLine2f>("DEBUG_ALL_POINTS");
//    debugValidPoints = writeChannel<lms::math::polyLine2f>("DEBUG_VALID_POINTS");
//    debugTranslatedPoints = writeChannel<lms::math::polyLine2f>("DEBUG_VALID_TRANSLATED_POINTS");

    // todo: we have not defined the interface for this yet
//    car = readChannel<street_environment::CarCommand>("CAR"); //TODO create ego-estimation service

    return true;
}

void NewRoadDetection::roadCallback(const drive_ros_image_recognition::RoadLaneConstPtr& road_in_) {
  // use simple singular buffer
  road_buffer_ = *road_in_;
}

void NewRoadDetection::imageCallback(const sensor_msgs::ImageConstPtr& img_in) {
    CvImagePtr img_conv = convertImageMessage(img_in);

    // todo: adjust this time value
    if (img_in->header.stamp.toSec() - road_buffer_.header.stamp.toSec() > 0.1) {
      ROS_WARN("Outdated road callback in buffer, skipping incoming image");
      return;
    }

    //if we have a road(?), try to find the line
    if(road_buffer_.points.size() > 1){
        find(img_conv); //TODO use bool from find
    }

    float dx = 0;
    float dy = 0;
    float dPhi = 0;

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
    return;
}

std::vector<cv::Point2f> NewRoadDetection::findBySobel(const bool renderDebugImage,
        const std::vector<int> &xv,
        const std::vector<int> &yv,
        const float minLineWidthMul,
        const float maxLineWidthMul,
        const float iDist,
        const float wDist,
        const int threshold)
{
  //    return ::findBySobel(image.get(),debugImage.get(),renderDebugImage,xv,yv,minLineWidthMul,maxLineWidthMul,0.02,iDist,wDist,threshold,homo);

  // todo: port this here
  //  inline std::vector<lms::math::vertex2f> findBySobel(
//          const lms::imaging::Image *image,
//          lms::imaging::Image *debugImage,
//          const bool renderDebugImage,
//          const std::vector<int> &xv,
//          const std::vector<int> &yv,
//          const float minLineWidthMul,
//          const float maxLineWidthMul,
//          const float lineWidth,
//          const float iDist,
//          const float wDist,
//          const int threshold,
//          lms::imaging::Transformation &trans) {
//      //lms::logging::Logger logger("findBySobel");

//      lms::imaging::BGRAImageGraphics graphics(*debugImage);
//      std::vector<lms::math::vertex2f> foundPoints;
//      bool foundLowHigh = false;
//      int pxlCounter = 0;
//      for(int k = 1; k < (int)xv.size()-1; k++){
//          const int xs = xv[k-1];
//          const int ys = yv[k-1];
//          const int x = xv[k];
//          const int y = yv[k];
//          const int xS = xv[k+1];
//          const int yS = yv[k+1];
//          //check if point is inside the image
//          if(!image->inside(xs,ys) || !image->inside(x,y) || !image->inside(xS,yS)){
//              continue;
//          }
//          if(renderDebugImage){
//              graphics.setColor(lms::imaging::blue);
//              graphics.drawCross(x,y);
//          }

//          int colors = *(image->data()+xs+ys*image->width());
//          int colorS = *(image->data()+xS+yS*image->width());
//          int sobel = colorS-colors;
//          //da wir von links nach rechts suchen ist positiver sobel ein dunkel-hell übergang
//          if(sobel > threshold){
//              if(!foundLowHigh){
//                  foundLowHigh = true;
//                  //logger.debug("")<<"crossing found lowHigh"<<std::endl;
//                  pxlCounter = 0;
//              }
//              if(renderDebugImage){
//                  graphics.setColor(lms::imaging::green);
//                  graphics.drawCross(x,y,3);
//              }
//          }else if(sobel < -threshold){ //hell-dunkel übergang
//              if(renderDebugImage){
//                  graphics.setColor(lms::imaging::yellow);
//                  graphics.drawCross(x,y,3);
//              }
//              //check if we found a lowHigh + highLow border
//              if(foundLowHigh){
//                  //check if the points have the right distance
//                  float pxlPeakWidth = iDist/wDist*lineWidth; //TODO to bad, calculate for each road line (how should we use them for searching?

//                  //logger.debug("")<<"crossing found highLow: "<<pxlCounter<<" "<<pxlPeakWidth;
//                  //logger.debug("")<<"crossing found max: "<<pxlPeakWidth*maxLineWidthMul;
//                  //logger.debug("")<<"crossing found min: "<<pxlPeakWidth*minLineWidthMul;

//                  if(pxlCounter > pxlPeakWidth*minLineWidthMul && pxlCounter < pxlPeakWidth*maxLineWidthMul){
//                      //we found a valid poit, mark it
//                      if(renderDebugImage){
//                          graphics.setColor(lms::imaging::red);
//                          for(int j = 0; j<pxlCounter;j++){
//                              graphics.drawCross(xv[k-j],yv[k-j]);
//                          }
//                      }
//                      //we found a valid point
//                      //get the middle
//                      lms::math::vertex2f wMid;
//                      trans.t(xv[k-pxlCounter/2],yv[k-pxlCounter/2],wMid.x,wMid.y);
//                      foundPoints.push_back(wMid);
//                      //logger.debug("")<<"crossing FOUND VALID CROSSING";
//                  }
//              }
//              if(renderDebugImage && pxlCounter > 0){
//                  graphics.setColor(lms::imaging::green);
//                  graphics.drawCross(x,y);
//              }
//              pxlCounter = 0;
//              foundLowHigh = false;
//              //if not, we dont have to do anything
//          }
//          if(foundLowHigh){
//              pxlCounter++;
//          }
//      }
//      return foundPoints;
//  }
}

std::vector<lms::math::vertex2f> NewRoadDetection::findByBrightness(const bool renderDebugImage, const std::vector<int> &xv,const std::vector<int> &yv, const float minLineWidthMul, const float maxLineWidthMul,const float iDist,const float wDist, const int threshold){
    lms::imaging::BGRAImageGraphics graphics(*debugImage);
    std::vector<lms::math::vertex2f> foundPoints;
    std::vector<int> color;
    //get the color from the points
    if(renderDebugImage)
        graphics.setColor(lms::imaging::blue);
    for(int k = 0; k < (int)xv.size(); k++){
        const int x = xv[k];
        const int y = yv[k];
        if(!image->inside(x,y)){
            color.push_back(0);
            continue;
        }
        color.push_back(*(image->data()+x+y*image->width()));
        if(renderDebugImage)
            graphics.drawCross(x,y);
    }
    //detect peaks
    float pxlPeakWidth = iDist/wDist*0.02; //TODO to bad, calculate for each road line (how should we use them for searching?
    int tCounter = 0;
    if(renderDebugImage)
        graphics.setColor(lms::imaging::red);
    for(int k = 0; k < (int)color.size(); k++){
        if(color[k]>threshold){
            tCounter++;
        }else{
            if(tCounter - k != 0 && tCounter > pxlPeakWidth*minLineWidthMul && tCounter < pxlPeakWidth*maxLineWidthMul){
                if(renderDebugImage){
                    for(int j = 0; j<tCounter;j++){
                        graphics.drawCross(xv[k-j],yv[k-j]);
                    }
                }
                //we found a valid point
                //get the middle
                lms::math::vertex2i mid;
                lms::math::vertex2f wMid;
                homo.vt(mid,wMid);
                foundPoints.push_back(wMid);
            }
            tCounter = 0;
        }
    }
    return foundPoints;
}

bool NewRoadDetection::find(CvImagePtr& incoming_image){
    //clear old lines
    ROS_INFO("Creating new lines");
    lines_.clear();
    linesToProcess_ = 0;

    //TODO rectangle for neglecting areas

    //(TODO calculate threshold for each line)
    if(renderDebugImage){
        //Clear debug image
        debugImage->resize(incoming_image->rows,incoming_image->cols);
    }

    //create left/mid/right lane
    // todo: port this function to ROS
    std::vector<cv::Point2f> mid = road->getWithDistanceBetweenPoints(config().get<float>("distanceBetweenSearchlines",0.2));
    std::vector<cv::Point2f> left = mid.moveOrthogonal(-0.4);
    std::vector<cv::Point2f> right = mid.moveOrthogonal(0.4);
    if(mid.size() != left.size() || mid.size() != right.size()){
      ROS_ERROR("Generated lane sizes do not match! Aborting search!");
      return false;
    }
    //get all lines
    for(int i = 0; i< mid.size(); i++){
        SearchLine l;
        l.w_start = left[i];
        l.w_end = right[i];
        l.w_left = left[i];
        l.w_mid = mid[i];
        l.w_right = right[i];
        //check if the part is valid (middle should be always visible)

        // todo: implement transformation functions from world to map and reversed
        // maybe this would be a good use for services (from the preprocessing node, has access to homography stuff anyway)
        int l_w_startx, l_w_starty, l_w_endx, l_w_endy;
        worldToImage(l.w_start.x, l.w_start.y, l_w_startx, l_w_starty);
        worldToImage(l.w_end.x, l.w_end.y, l_w_endx, l_w_endy);
        cv::Rect img_rect(cv::Point(),cv::Point(incoming_image->cols,incoming_image->rows));
        if(!img_rect.contains(cv::Point(l_w_startx,l_w_starty))){
            // try middle lane -> should always be in image
            l.w_start = mid[i];
            worldToImage(l.w_start.x, l.w_start.y, l_w_startx, l_w_starty);
            if(img_rect.contains(cv::Point(l_w_startx,l_w_starty))){
                continue;
            }
        }else if(!img_rect.contains(cv::Point(l_w_endx,l_w_endy))){
            l.w_end = mid[i];
        }

        //transform them in image-coordinates
        std::unique_lock<std::mutex> lock(mutex);
        lines_.push_back(l);
        linesToProcess_++;

        // todo: figure out what this does (no internet)
//        conditionNewLine.notify_one();
    }

    if(numThreads_ == 0) {
        // single threaded
        for(SearchLine &l:lines_){
            processSearchLine(l);
        }
    } else {
        // multi threaded

        // initialize and start threads if not yet there
        if(threads_.size() == 0) {
            threadsRunning_ = true;
            for(int i = 0; i < numThreads_; i++) {
                threads_.emplace_back([this] () {
                    threadFunction();
                });
            }
        }

        // wait till every search line was processed
        {
            std::unique_lock<std::mutex> lock(mutex);
            while(linesToProcess_ > 0) conditionNewLine_.wait(lock);
        }
    }
    return true;
}

void NewRoadDetection::threadFunction() {
    while(threadsRunning_) {
        SearchLine line;
        {
            std::unique_lock<std::mutex> lock(mutex);
            while(threadsRunning_ && lines_.empty()) conditionNewLine_.wait(lock);
            if(lines_.size() > 0) {
                line = lines_.front();
                lines_.pop_front();
            }
            if(!threadsRunning_) {
                break;
            }
        }
        processSearchLine(line);
    }
}

void NewRoadDetection::processSearchLine(const SearchLine &l) {
    std::vector<int> xv;
    std::vector<int> yv;

    //calculate the offset
    float iDist = l.i_start.distance(l.i_end);
    float wDist = l.w_start.distance(l.w_end);
    float pxlPerDist = iDist/wDist*searchOffset;
    lms::math::vertex2f iDiff = lms::math::vertex2f(l.i_start-l.i_end).normalize();
    lms::math::vertex2f startLine = lms::math::vertex2f(l.i_start)+iDiff*pxlPerDist;
    lms::math::vertex2f endLine = lms::math::vertex2f(l.i_end)-iDiff*pxlPerDist;
    //get all points in between
    lms::math::bresenhamLine(startLine.x,startLine.y,endLine.x,endLine.y,xv,yv); //wir suchen von links nach rechts!

    //find points
    std::vector<lms::math::vertex2f> foundPoints;
    if(findPointsBySobel){
       foundPoints = findBySobel(renderDebugImage,xv,yv,minLineWidthMul,maxLineWidthMul,iDist,wDist,sobelThreshold);
    }else{
       foundPoints = findByBrightness(renderDebugImage,xv,yv,minLineWidthMul,maxLineWidthMul,iDist,wDist,threshold);
    }

    if(renderDebugImage) {
        std::unique_lock<std::mutex> lock(debugAllPointsMutex);
        for(lms::math::vertex2f &v:foundPoints)
            debugAllPoints->points().push_back(v);
    }

    //filter
    //TODO filter points that are in poluted regions (for example the car itself)
    //remove points with bad distances
    if(foundPoints.size() > 2){
        std::vector<int> foundCounter;
        foundCounter.resize(foundPoints.size(),0);
        for(std::size_t fp = 0; fp < foundPoints.size(); fp++){
            const lms::math::vertex2f &s = foundPoints[fp];
            for(std::size_t test = fp+1; test <foundPoints.size(); test++){
                const lms::math::vertex2f &toTest  = foundPoints[test];
                float distance = toTest.distance(s);
                if((distance > 0.4-laneWidthOffsetInMeter && distance < 0.4 + laneWidthOffsetInMeter)|| (distance > 0.8-laneWidthOffsetInMeter && distance < 0.8+laneWidthOffsetInMeter)){
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
        std::vector<lms::math::vertex2f> validPoints;
        for(int i = 0; i < (int)foundPoints.size(); i++){
            if(foundCounter[i] >= max){
                validPoints.push_back(foundPoints[i]);
            }
        }
        //TODO if we have more than 3 points we know that there is an error!
        foundPoints = validPoints;
    }

    if(renderDebugImage){
        std::unique_lock<std::mutex> lock(debugValidPointsMutex);
        for(lms::math::vertex2f &v:foundPoints)
            debugValidPoints->points().push_back(v);
    }

    //Handle found points
    lms::math::vertex2f diff;
    std::vector<float> distances;
    for(int i = 0; i < (int)foundPoints.size(); i++){
        lms::math::vertex2f &wMind = foundPoints[i];
        float distanceToLeft = wMind.distance(l.w_left);
        float distanceToRight= wMind.distance(l.w_right);
        float distanceToMid= wMind.distance(l.w_mid);
        diff = (l.w_left-l.w_right).normalize();
        if(distanceToLeft < distanceToMid && distanceToLeft < distanceToRight){
            //left point
            wMind-=diff*0.4;
            distances.push_back(distanceToLeft);
        }else if(distanceToRight < distanceToMid ){
            wMind+=diff*0.4;
            distances.push_back(distanceToRight);
        }else{
            distances.push_back(distanceToMid);
        }
    }

    std::vector<float> weights;
    for(const float &dist:distances){
        //as distance is in meter, we multiply it by 100
        if(useWeights)
            weights.push_back(1/(dist*100+0.001)); //TODO hier etwas sinnvolles überlegen
        else
            weights.push_back(1);
    }
    /*
    if(renderDebugImage){
        for(lms::math::vertex2f &v:foundPoints)
            debugTranslatedPoints->points().push_back(v);
    }
    */
    lms::ServiceHandle<local_course::LocalCourse> localCourse = getService<local_course::LocalCourse>("LOCAL_COURSE_SERVICE");
    if(localCourse.isValid()){
        localCourse->addPoints(foundPoints,weights);
    }else{
        logger.error("localCourse invalid!");
    }

    {
        std::unique_lock<std::mutex> lock(mutex);
        linesToProcess --;
        conditionLineProcessed.notify_all();
    }
}

void NewRoadDetection::reconfigureCB(drive_ros_image_recognition::new_road_detectionConfig& config, uint32_t level){
    searchOffset_ = config.searchOffset;
    findPointsBySobel_ = config.findBySobel;
    renderDebugImage_ = config.renderDebugImage;
    minLineWidthMul_ = config.minLineWidthMul;
    maxLineWidthMul_ = config.maxLineWidthMul;
    threshold_ = config.threshold;
    laneWidthOffsetInMeter_ = config.laneWidthOffsetInMeter;
    useWeights_ = config.useWeights;
    sobelThreshold_ = config.sobelThreshold;
    numThreads_ = config.threads;
}

NewRoadDetection::~NewRoadDetection() {
    {
        std::unique_lock<std::mutex> lock(mutex);
        threadsRunning = false;
        conditionNewLine.notify_all();
    }
    for(auto &t : threads) {
        if(t.joinable()) {
            t.join();
        }
    }
}
