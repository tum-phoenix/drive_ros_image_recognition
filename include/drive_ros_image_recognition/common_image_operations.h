#ifndef COMMON_IMAGE_OPERATIONS_H
#define COMMON_IMAGE_OPERATIONS_H

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <drive_ros_image_recognition/NewRoadDetectionConfig.h>
#include <sensor_msgs/CameraInfo.h>

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

inline void cam_info_sub(const sensor_msgs::CameraInfoConstPtr& incoming_cam_info, image_geometry::PinholeCameraModel& cam_model) {
    cam_model.fromCameraInfo(incoming_cam_info);
    ROS_INFO("Received camera info callback, processing for use in TransformHelper");
}

inline bool getHomographyMatParam(const ros::NodeHandle& pnh, cv::Mat mat, const std::string mat_param) {
    // retrieve world2map and map2world homography matrices
    std::vector<double> temp_vals;

    if (!pnh.getParam("homography_matrix/"+mat_param, temp_vals)) {
      ROS_ERROR("Unable to load homography matrix %s from configuration file!", mat_param.c_str());
      return false;
    }
    if (temp_vals.size() != 9) {
      ROS_ERROR("Retreived homography matrix %s does not have 9 values", mat_param.c_str());
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

enum search_method {
    sobel = false,
    brightness = true
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
        cam2world_(),
        cam_info_sub_()
    {
    }

    TransformHelper& operator=(TransformHelper&& other)
    {
        std::swap(cam_model_, other.cam_model_);
        std::swap(cam2world_, other.cam2world_);
        // tf listener also not swappable, should auto-initialize
#ifdef USE_WORLD2CAM_HOMOGRAPHY
        std::swap(world2cam_, other.world2cam_);
#endif
        // cannot swap subscriber, need to bind to new object
        ros::NodeHandle nh;
        cam_info_sub_ = nh.subscribe<sensor_msgs::CameraInfo>(other.cam_info_sub_.getTopic(), 1,
                                                                boost::bind(&cam_info_sub, _1,
                                                                            getCameraModelReference()) );
        return *this;
    }

    // todo: properly reinitialize subscriber with new binding to this object
    TransformHelper(const TransformHelper& helper):
    tf_listener_()
    {
        cam2world_ = helper.cam2world_;
#ifdef USE_WORLD2CAM_HOMOGRAPHY
        world2cam_ = helper.world2cam_;
#endif
        cam_model_ = helper.cam_model_;
        ros::NodeHandle nh;
        cam_info_sub_ = nh.subscribe<sensor_msgs::CameraInfo>(helper.cam_info_sub_.getTopic(), 1,
                                                              boost::bind(&cam_info_sub, _1,
                                                                          getCameraModelReference()) );
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
        tf_listener_(),
        cam_info_sub_()
    {
    }

    ~TransformHelper() {
    }

    bool init(ros::NodeHandle& pnh) {
#ifdef USE_WORLD2CAM_HOMOGRAPHY
        if (!getHomographyMatParam(pnh, world2cam_, "world2cam"))
            return false;
#endif

        if (!getHomographyMatParam(pnh, cam2world_, "cam2world"))
            return false;

        // subscribe Camera model to TransformHelper-> this is kind of hacky, but should keep the camera model on the transform helper updated
        cam_info_sub_ = pnh.subscribe<sensor_msgs::CameraInfo>("/camera1/camera_info", 1,
                                                                boost::bind(&cam_info_sub, _1,
                                                                            getCameraModelReference()) );
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

    void setcam2worldMat(const cv::Mat& cam2world) {
        cam2world_ = cam2world;
    }

    // to subscribe to camerainfo messages
    image_geometry::PinholeCameraModel& getCameraModelReference() {
        return cam_model_;
    }

#ifdef USE_WORLD2CAM_HOMOGRAPHY
    void setworld2camMat(const cv::Mat& world2cam) {
        world2cam_ = world2cam;
    }
#endif

private:
    image_geometry::PinholeCameraModel cam_model_;
    cv::Mat cam2world_;
    tf::TransformListener tf_listener_;
#ifdef USE_WORLD2CAM_HOMOGRAPHY
    cv::Mat world2cam_;
#endif
    ros::Subscriber cam_info_sub_;
}; // class TransformHelper

class ImageOperator {

public:

    ImageOperator():
        helper_()
    {
    }

    ImageOperator(ImageOperator& image_operator):
        helper_(image_operator.helper_),
        config_(image_operator.config_)
    {
    }

    ImageOperator& operator=(ImageOperator&& other)
    {
        std::swap(helper_, other.helper_);
        std::swap(config_, other.config_);
        return *this;
    }

    ImageOperator(TransformHelper& helper):
        helper_(helper)
    {
    }

    std::vector<cv::Point2f> returnValidPoints(const SearchLine &sl,
                                         const cv::Mat& current_image,
                                         const float lineWidth,
                                         const search_direction search_dir,
                                         const search_method method)
    {
        std::vector<cv::Point> imagePoints;
        std::vector<int> lineWidths;
        std::vector<cv::Point2f> foundPoints;

        float iDist = cv::norm(sl.iEnd - sl.iStart);
        float wDist = cv::norm(sl.wEnd - sl.wStart);
        float pxlPeakWidth = iDist/wDist*lineWidth;
        findByLineSearch(sl, current_image, search_dir, method, imagePoints, lineWidths);

        cv::Point2f wMid;

        for (int i=0; i<imagePoints.size(); ++i) {
            if (lineWidths[i] > pxlPeakWidth*config_.minLineWidthMul && lineWidths[i] < pxlPeakWidth*config_.maxLineWidthMul) {
                helper_.imageToWorld(imagePoints[i], wMid);
                foundPoints.push_back(wMid);
            }
        }
        return foundPoints;
    }

    void findByLineSearch(const SearchLine& sl,
                          const cv::Mat& current_image,
                          const search_direction search_dir,
                          const search_method search_meth,
                          std::vector<cv::Point>& image_points,
                          std::vector<int>& line_widths
                          )
    {
#ifdef DRAW_DEBUG
            if (config_.renderDebugImage) {
                cv::Mat debug_image = current_image.clone();
                cv::namedWindow("Unfiltered points processed by brightness search", CV_WINDOW_NORMAL);
                cv::circle(debug_image, it.pos(), 2, cv::Scalar(255));
                cv::imshow("Unfiltered points processed by brightness search", debug_image);
            }
#endif

        // step 1: threshold image with mat:
        if (search_meth == search_method::brightness)
            cv::threshold( current_image, current_image, config_.brightness_threshold, 255, CV_THRESH_BINARY );
        else if (search_meth == search_method::sobel) {
            if (search_dir == search_direction::x)
                cv::Sobel( current_image, current_image, CV_8UC1, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
            else if(search_dir == search_direction::y) {
                cv::Sobel( current_image, current_image, CV_8UC1, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);
            }
        }

#ifdef DRAW_DEBUG
        // thresholded image
        if (config_.renderDebugImage) {
            cv::Mat debug_image = current_image.clone();
            cv::namedWindow("Thresholded image in brightness search", CV_WINDOW_NORMAL);
            cv::circle(debug_image, it.pos(), 2, cv::Scalar(255));
            cv::imshow("Thresholded image in brightness search", debug_image);
        }
#endif
        // step 2: line mask to get only relevant pixels in search area
        cv::Mat line_mat(current_image.rows, current_image.cols, CV_8UC1, cv::Scalar(0));
        cv::line(line_mat, sl.iStart, sl.iEnd, cv::Scalar(255));

        // step3: binary threshold with line mask
        cv::Mat overlay_mat;
        cv::bitwise_and(current_image, line_mat, overlay_mat);

#ifdef DRAW_DEBUG
        // line thresholded image
        if (config_.renderDebugImage) {
            cv::namedWindow("Line mask overlay", CV_WINDOW_NORMAL);
            cv::imshow("Line mask overlay", overlay_mat);
        }
#endif

        // step4: get line thickness from image
        int line_thickness = (int)cv::sum(overlay_mat)[0];

        // step5: get midpoint of line
        cv::Mat locations;
        cv::findNonZero(overlay_mat, locations);
        double min, max;
        if (search_dir == search_direction::x) {
            cv::minMaxLoc(locations.col(0), &min, &max);
        } else if (search_dir == search_direction::y) {
            cv::minMaxLoc(locations.col(1), &min, &max);
        }

        if (search_dir == search_direction::x) {
            line_widths.push_back(max-min);
            image_points.push_back(cv::Point(min+line_thickness*0.5, sl.iStart.y));
        } else if (search_dir == search_direction::y) {
            line_widths.push_back(max-min);
            image_points.push_back(cv::Point(sl.iStart.x,min+line_thickness*0.5));
        }
        return;
    }

    // debug to perform line check
    void debugPointsImage (const cv::Mat& current_image,
                           const search_direction search_dir,
                           const search_method search_meth
                           ) {
        std::vector<cv::Point> image_points;
        std::vector<int> image_widths;
        cv::Mat debug_image;
        cv::cvtColor(current_image, debug_image, CV_GRAY2RGB);

        int search_steps = 5;
        int pixel_step = 50;
        SearchLine lines[search_steps];
        for (int it = 0; it < search_steps*pixel_step; it+=pixel_step) {
            if (search_dir == search_direction::x) {
                lines[it].iStart = cv::Point(0,it*pixel_step);
                lines[it].iEnd = cv::Point(current_image.rows, it*pixel_step);
            } else if (search_dir == search_direction::y) {
                lines[it].iStart = cv::Point(it*pixel_step, 0);
                lines[it].iEnd = cv::Point(it*pixel_step, current_image.cols);
            }
            cv::line(debug_image, lines[it].iStart, lines[it].iEnd, cv::Scalar(0,255,0));
            findByLineSearch(lines[it],
                             current_image,
                             search_dir,
                             search_meth,
                             image_points,
                             image_widths
                             );
            for (auto point: image_points)
                cv::circle(debug_image,point,2,cv::Scalar(255,0,0),2);
        }
        cv::namedWindow("Debug lineCheck", CV_WINDOW_NORMAL);
        cv::imshow("Debug lineCheck", debug_image);
        cv::waitKey(0);
        return;
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
