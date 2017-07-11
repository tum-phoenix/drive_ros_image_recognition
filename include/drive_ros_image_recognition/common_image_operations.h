#ifndef COMMON_IMAGE_OPERATIONS_H
#define COMMON_IMAGE_OPERATIONS_H

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <drive_ros_image_recognition/NewRoadDetectionConfig.h>

typedef boost::shared_ptr<cv::Mat> CvImagePtr;

inline CvImagePtr convertImageMessage(const sensor_msgs::ImageConstPtr& img_in) {
    CvImagePtr cv_ptr;
    try
    {
        // hardcopies for now, might be possible to process on pointer if fast enough
        // todo: make prettier
        cv_bridge::CvImagePtr temp_ptr = cv_bridge::toCvCopy(*img_in, "");
        cv_ptr.reset(new cv::Mat(temp_ptr->image) );
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return NULL;
    }

    if( !cv_ptr->data)
    {
        ROS_WARN("Empty image received, skipping!");
        return NULL;
    }
    return cv_ptr;
}


namespace drive_ros_image_recognition {

inline bool getHomographyMatParam(const ros::NodeHandle& pnh, cv::Mat mat, const std::string mat_param) {
    // retrieve world2map and map2world homography matrices
    std::vector<double> temp_vals;

    if (!pnh.getParam("homography_matrix/"+mat_param, temp_vals)) {
      ROS_ERROR("Unable to load homography matrix from configuration file!");
      return false;
    }
    if (temp_vals.size() != 9) {
      ROS_ERROR("Retreived homography matrix does not have 9 values!");
      return false;
    }
    for(unsigned i=0; i < temp_vals.size(); i++) {
      // todo: check: according to warp_cpp.h, those two might actually be the other way round
      mat.at<double>(i) = temp_vals[i];
    }
    ROS_DEBUG_STREAM("Paramater matrix loaded as: "<<mat);
    return true;
}

enum search_direction {
    x = false,
    y = true
};

struct SearchLine{
    cv::Point2f wStart;
    cv::Point2f wEnd;
    cv::Point2i iStart;
    cv::Point2i iEnd;

    cv::Point2f wLeft;
    cv::Point2f wMid;
    cv::Point2f wRight;
};

class TransformHelper {

public:

    TransformHelper():
        tf_listener_(),
    #ifdef USE_WORLD2CAM_HOMOGRAPHY
        world2cam_(),
    #endif
        cam_model_(),
        cam2world_()
    {
    }

    TransformHelper(const TransformHelper& helper):
    tf_listener_()
    {
        cam2world_ = helper.cam2world_;
#ifdef USE_WORLD2CAM_HOMOGRAPHY
        world2cam_ = helper.world2cam_;
#endif
        cam_model_ = helper.cam_model_;
    }

    TransformHelper(const cv::Mat& cam2world, const image_geometry::PinholeCameraModel& cam_model
                #ifdef USE_WORLD2CAM_HOMOGRAPHY
                    const cv::Mat& world2cam
                #endif
                    ):
        cam2world_(cam2world),
    #ifdef USE_WORLD2CAM_HOMOGRAPHY
        world2cam_(world2cam),
    #endif
        cam_model_(cam_model),
        tf_listener_()
    {
    }

    ~TransformHelper() {
    }

    void setCamModel(const image_geometry::PinholeCameraModel& cam_model) {
        cam_model_ = cam_model;
    }

    bool worldToImage(const cv::Point3f& world_point,
                      cv::Point& image_point) {
#ifdef USE_WORLD2CAM_HOMOGRAPHY
        // using homography instead of camera model
        cv::perspectiveTransform(world_point, image_point, world2cam_);
        if (point_world.rows != 1 || point_world.cols != 1) {
            ROS_WARN("Point transformed to image dimensions has invalid dimensions");
            return false;
        }
#else
        image_point = cam_model_.project3dToPixel(world_point);
#endif
        return true;
    }

    bool worldToImage(const cv::Point2f &world_point, cv::Point &image_point) {
        return worldToImage(cv::Point3f(world_point.x, world_point.y, 0.0f), image_point);
    }

    bool imageToWorld(const cv::Point& image_point, cv::Point2f& world_point) {
        cv::Mat mat_point_image(1,1,CV_64FC2);
        mat_point_image.at<double>(0,0) = image_point.x;
        mat_point_image.at<double>(0,1) = image_point.y;
        cv::Mat mat_point_world(1,1,CV_64FC2);
        cv::perspectiveTransform(mat_point_image, mat_point_world, cam2world_);
        cv::Point2f cam_point(mat_point_world.at<float>(0,0),mat_point_world.at<float>(0,1));

        // todo: check if it is valid to assume the z depth if we used homography before
        cv::Point3d cam_ray_point = cam_model_.projectPixelTo3dRay(cam_point);

        geometry_msgs::PointStamped camera_point;
        camera_point.header.frame_id = cam_model_.tfFrame();
        camera_point.point.x = cam_ray_point.x;
        camera_point.point.y = cam_ray_point.y;
        camera_point.point.z = cam_ray_point.z;
        geometry_msgs::PointStamped geom_world_point;

        try {
            ros::Duration timeout(1.0 / 30);
            tf_listener_.waitForTransform(cam_model_.tfFrame(), "tf_front_axis_middle",
                                          ros::Time::now(), timeout);
            tf_listener_.transformPoint("tf_front_axis_middle", camera_point, geom_world_point);
        }
        catch (tf::TransformException& ex) {
            ROS_WARN("[ImageToWorld] TF exception, unable to transform:\n%s", ex.what());
            return false;
        }
        world_point.x = geom_world_point.point.x;
        world_point.y = geom_world_point.point.y;
        return true;
    }

private:
    image_geometry::PinholeCameraModel cam_model_;
    cv::Mat cam2world_;
    tf::TransformListener tf_listener_;
#ifdef USE_WORLD2CAM_HOMOGRAPHY
    cv::Mat world2cam_;
#endif
}; // class TransformHelper

//std::vector<cv::Point2f> processSearchLine(SearchLine &sl, const cv::Mat& current_image, const search_direction search_dir) {
//    std::vector<cv::Point2f> foundPoints;
//    bool foundLowHigh = false;
//    int pxlCounter = 0;
//    cv::Point2f iStartPxlPeak;

//    cv::LineIterator it(*currentSobelImage, sl.iStart, sl.iEnd);

//    found_points = findBySobel(lineIt, current_image, lineWidth_, iDist, wDist, search_dir);
//    cv::Rect rect(0, 0, currentSobelImage->cols, currentSobelImage->rows);
//    //  float iDist = cv::norm(sl.iEnd - sl.iStart);
//    //  float wDist = cv::norm(sl.wEnd - sl.wStart);

//    for(int i = 0; i < lineIt.count; i++, ++lineIt) {
//        // safety check : is the point inside the image
//        if(!rect.contains(lineIt.pos())){

//            ROS_WARN("Received an invalid point outside the image to check for Sobel (%i,%i)", lineIt.pos().x, lineIt.pos().y);
//            return foundPoints;
//        }



// Search for a bright pixel. Searching from bottom to top

//        int sobel = currentSobelImage->at<char>(lineIt.pos());

//        if(sobel > sobelThreshold) {
//            // found low-high pass
//            if(!foundLowHigh) {
//                foundLowHigh = true;
//                pxlCounter = 0;
//                iStartPxlPeak.x = lineIt.pos().x;
//                iStartPxlPeak.y = lineIt.pos().y;
//            }
//        } else if(sobel < -sobelThreshold) {
//            // found high-low pass
//            // check if we found a lowHigh + highLow border
//            if(foundLowHigh){
//                //check if the points have the right distance
//                // TODO to bad, calculate for each road line (how should we use them for searching?
//                // float pxlPeakWidth = iDist / wDist * lineWidth; // todo: understand this

//                //logger.debug("")<<"crossing found highLow: "<<pxlCounter<<" "<<pxlPeakWidth;
//                //logger.debug("")<<"crossing found max: "<<pxlPeakWidth*maxLineWidthMul;
//                //logger.debug("")<<"crossing found min: "<<pxlPeakWidth*minLineWidthMul;

//                // todo: check if the line we found has the correct height
//                // if((pxlCounter > (pxlPeakWidth * minLineWidthMul)) && (pxlCounter < (pxlPeakWidth * maxLineWidthMul))) {
//                // return the middle of the line we found
//                // cv::Point2f iMid((iStartPxlPeak.x + lineIt.pos().x) *.5, (iStartPxlPeak.x + lineIt.pos().y) * .5);

//                foundPoints.push_back(lineIt.pos());
//                // cv::Point2f wMid;
//                // imageToWorld(iMid, wMid);
//                // foundPoints.push_back(wMid);
//                // ROS_DEBUG("Found a stop-line");
//                // }
//            }
//            pxlCounter = 0;
//            foundLowHigh = false;
//            // if not, we dont have to do anything
//        }

//        // for calculation of line width
//        if(foundLowHigh){
//            pxlCounter++;
//        }
//    }
//    return foundPoints;
//}

class ImageOperator {

public:

    ImageOperator(TransformHelper& helper):
        helper_(helper)
    {
    }

    std::vector<cv::Point2f> findBySobel(const SearchLine &sl,
                                         const cv::Mat& current_image,
                                         const float lineWidth,
                                         const float iDist,
                                         const float wDist,
                                         const search_direction search_dir)
    {
        std::vector<cv::Point2f> foundPoints;
        bool foundLowHigh = false;
        int pxlCounter = 0;
        cv::Mat current_image_sobel(current_image.rows,current_image.cols,CV_8UC1,cv::Scalar(0));
        cv::LineIterator it(current_image_sobel, sl.iStart, sl.iEnd);
        cv::LineIterator it_backup = it;
        if (search_dir == search_direction::x)
            cv::Sobel( current_image, current_image_sobel, CV_8UC1, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
        else if(search_dir == search_direction::y) {
            // todo: in Simon's code this used a kernel size of 1 which did not really make sense, but check if this is valid anyway
            cv::Sobel( current_image, current_image_sobel, CV_8UC1, -1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
        }

        cv::Rect rect(cv::Point(),cv::Point(current_image.rows, current_image.cols));

        for(int i = 0; i < it.count; i++, ++it)
        {
            // safety check : is the point inside the image
            if(!rect.contains(it.pos())){
                ROS_WARN("Received an invalid point outside the image to check for Sobel");
                return foundPoints;
            }

            //da wir von links nach rechts suchen ist positiver sobel ein dunkel-hell übergang
            int sobel = (int)current_image_sobel.at<unsigned char>(it.pos().x,it.pos().y);
            if(sobel > config_.sobelThreshold){
                // found low-high pass (on the left side of the line)
                if(!foundLowHigh){
                    foundLowHigh = true;
                    pxlCounter = 0;
                }
            }else if(sobel < -config_.sobelThreshold){ //hell-dunkel übergang
                //check if we found a lowHigh + highLow border
                if(foundLowHigh){
                    //check if the points have the right distance
                    float pxlPeakWidth = iDist/wDist*lineWidth; //TODO to bad, calculate for each road line (how should we use them for searching?

                    ROS_DEBUG_STREAM("crossing found highLow: "<<pxlCounter<<" "<<pxlPeakWidth);
                    ROS_DEBUG_STREAM("crossing found max: "<<pxlPeakWidth*config_.maxLineWidthMul);
                    ROS_DEBUG_STREAM("crossing found min: "<<pxlPeakWidth*config_.minLineWidthMul);

                    // basically returns the middle of a line from multiple points, so will have to loop through the points here
                    if(pxlCounter > pxlPeakWidth*config_.minLineWidthMul && pxlCounter < pxlPeakWidth*config_.maxLineWidthMul){
                        //we found a valid point
                        //get the middle
                        // todo: make this more elegant in both sobel and brightness (no std::advance for lineIterator)
                        for (int iter = 0; iter<(i-pxlCounter/2); iter++, ++it_backup) {
                        }
                        cv::Point2f wMid;
                        helper_.imageToWorld(it_backup.pos(),wMid);
                        foundPoints.push_back(wMid);
                        ROS_DEBUG("crossing FOUND VALID CROSSING");
                    }
                }
                pxlCounter = 0;
                foundLowHigh = false;
                //if not, we dont have to do anything
            }

            // for calculation of line width
            if(foundLowHigh){
                pxlCounter++;
            }
        }
        return foundPoints;
    }

    std::vector<cv::Point2f> findByBrightness( const SearchLine &sl,
                                               const cv::Mat& current_image,
                                               const float lineWidth,
                                               const float iDist,
                                               const float wDist,
                                               const search_direction search_dir)
    {
        std::vector<cv::Point2f> foundPoints;
        std::vector<int> color;
        cv::Rect rect(cv::Point(),cv::Point(current_image.rows, current_image.cols));
        cv::LineIterator it(current_image, sl.iStart, sl.iEnd);
        cv::LineIterator it_debug = it;

        for(int i = 0; i < it.count; i++, ++it) {
            // safety check : is the point inside the image
            if(!rect.contains(it.pos())){
                ROS_WARN("Received an invalid point outside the image to check for brightness");
                color.push_back(0);
                continue;
            }
            color.push_back(current_image.at<int>(it.pos().x,it.pos().y));

#ifdef DRAW_DEBUG
            if (config_.renderDebugImage) {
                cv::Mat debug_image = current_image.clone();
                cv::namedWindow("Unfiltered points processed by brightness search", CV_WINDOW_NORMAL);
                cv::circle(debug_image, it.pos(), 2, cv::Scalar(255));
                cv::imshow("Unfiltered points processed by brightness search", debug_image);
            }
#endif
        }

        //detect peaks
        float pxlPeakWidth = iDist/wDist*lineWidth; //TODO to bad, calculate for each road line (how should we use them for searching?
        int tCounter = 0;
        // todo: make this more efficient
        for(int k = 0; k < (int)color.size(); k++){
            if(color[k]>config_.brightness_threshold){
                tCounter++;
            }else{
                if(tCounter - k != 0 && tCounter > pxlPeakWidth*config_.minLineWidthMul && tCounter < pxlPeakWidth*config_.maxLineWidthMul){
                    for (int i = 0; i < tCounter; i++, ++it_debug) {
                    }
                    // get points for debug drawing on the way
                    cv::Point point_first = it_debug.pos();
                    for (int i = 0; i < (k-tCounter)/2; i++, ++it_debug) {
                    }
                    cv::Point point_mid = it_debug.pos();
                    for (int i = 0; i < (k-tCounter)/2; i++, ++it_debug) {
                    }
                    cv::Point point_end = it_debug.pos();

#ifdef DRAW_DEBUG
                    if (config_.renderDebugImage) {
                        cv::namedWindow("Line detected by brightness", CV_WINDOW_NORMAL);
                        cv::Mat debug_image = current_image.clone();

                        cv::line(debug_image, point_first, point_end, cv::Scalar(255));
                        cv::imshow("Line detected by brightness", debug_image);
                    }
#endif

                    //we found a valid point
                    //get the middle
                    cv::Point2f wMid;
                    helper_.imageToWorld(point_mid,wMid);
                    foundPoints.push_back(wMid);
                }
                tCounter = 0;
            }
        }
        return foundPoints;
    }

    void setConfig(const NewRoadDetectionConfig& config) {
        config_ = config;
    }

private:
    TransformHelper helper_;
    NewRoadDetectionConfig config_;

}; // class ImageOperator

} // end namespace

#endif // COMMON_IMAGE_OPERATIONS_H
