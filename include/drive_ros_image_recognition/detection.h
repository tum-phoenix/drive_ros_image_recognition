#ifndef DETECTION_H
#define DETECTION_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <drive_ros_image_recognition/geometry_common.h>
#include <drive_ros_image_recognition/common_image_operations.h>
#include <nodelet/nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/subscriber_filter.h>
#include <dynamic_reconfigure/server.h>
#include <drive_ros_msgs/RoadLane.h>

namespace drive_ros_image_recognition {

template<class Config> class Detection : public nodelet::Nodelet {
protected:
    dynamic_reconfigure::Server<Config> dsrv_server_;
    typename dynamic_reconfigure::Server<Config>::CallbackType dsrv_cb_;
    Config config_;

    // subscribers and publishers
    std::unique_ptr<image_transport::ImageTransport> it_;
    // standalone image subscriber, internally generates hints
    image_transport::Subscriber img_sub_standalone_;
    std::unique_ptr<image_transport::SubscriberFilter> img_sub_;
    std::unique_ptr<message_filters::Subscriber<drive_ros_msgs::RoadLane> > road_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, drive_ros_msgs::RoadLane> SyncImageToRoad;
    std::unique_ptr<message_filters::Synchronizer<SyncImageToRoad> > sync_;
#ifdef PUBLISH_DEBUG
    image_transport::Publisher visualization_pub_;
#endif

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    CvImagePtr current_image_;
    std::vector<geometry_msgs::PointStamped> road_hints_buffer_;
    ImageOperator image_operator_;

    virtual void imageCallback(const sensor_msgs::ImageConstPtr& img_in) = 0;
    virtual void syncCallback(const sensor_msgs::ImageConstPtr& img_in, const drive_ros_msgs::RoadLaneConstPtr& road_in) {
      current_image_ = convertImageMessage(img_in);
      road_hints_buffer_ = road_in->points;
      find();
    }
    virtual bool find() = 0;
    virtual void reconfigureCB(Config& config, uint32_t level) {config_ = config;}

public:
    Detection() : Detection(ros::NodeHandle(), ros::NodeHandle("~"), nullptr) {}
    Detection(const ros::NodeHandle nh, const ros::NodeHandle pnh, image_transport::ImageTransport* it) : nh_(nh), pnh_(pnh), image_operator_(), dsrv_server_(),
      dsrv_cb_(boost::bind(&Detection::reconfigureCB, this, _1, _2)), road_hints_buffer_(0), current_image_() { it_.reset(it); dsrv_server_.setCallback(dsrv_cb_);}
    ~Detection() {}
    virtual bool init() {
      image_operator_ = ImageOperator();
      if (!image_operator_.init(nh_, pnh_)) {
        ROS_ERROR("Failed to initialize ImageOperator, shutting down!");
        return false;
      }

#ifdef PUBLISH_DEBUG
      visualization_pub_ = it_->advertise("crosswalk_visualization_pub", 10);
#endif

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

      // sync callback registration - disabled for now as we are not publishing a road yet
      //  img_sub_.reset(new image_transport::SubscriberFilter(imageTransport_,"/warped_image", 5));
      //  road_sub_.reset(new message_filters::Subscriber<drive_ros_msgs::RoadLane>(pnh_,"/road_detection/road_in", 5));
      //  sync_.reset(new message_filters::Synchronizer<SyncImageToHints>(SyncImageToHints(5), *img_sub_, *road_sub_));
      //  sync_->registerCallback(boost::bind(&Detection::syncCallback, this, _1, _2));

      // just the image callback - used for debugging purposes
      img_sub_standalone_ = it_->subscribe("/warped_image", 1000, &Detection::imageCallback, this);
      return true;
    }
    virtual void onInit() {
      nh_ = getNodeHandle();
      pnh_ = getPrivateNodeHandle();
      it_.reset(new image_transport::ImageTransport(pnh_));
      road_hints_buffer_ = std::vector<geometry_msgs::PointStamped>(0);
      if (!init()) {
        ROS_ERROR("Nodelet failed to initialize");
        // nodelet failing will kill the entire loader anyway
        ros::shutdown();
      }
      else {
        ROS_INFO("Nodelet succesfully initialized");
      }
    }
};

}
#endif // DETECTION_H
