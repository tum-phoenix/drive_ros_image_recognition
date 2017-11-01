#include <gtest/gtest.h>
#include "drive_ros_image_recognition/common_image_operations.h"
#include <ros/ros.h>

// Declare test
TEST(CommonImageOperations, testCase0)
{
  ros::NodeHandle pnh("~");
  std::string world_frame("/rear_axis_middle_ground");
  if (!pnh.getParam("world_frame", world_frame))
    ROS_WARN_STREAM("Parameter 'world_frame' not found, using default: "<<world_frame);
  std::string camera_frame("/camera_optical");
  if (!pnh.getParam("camera_frame", camera_frame))
    ROS_WARN_STREAM("Parameter 'camera_frame' not found, using default: "<<camera_frame);
  const double err=2.0; // allowed error

  // transform an image point to the world and back again and verify we get the same results
  cv::Point image_point(100,280);
  cv::Point2f world_point_border;
  drive_ros_image_recognition::ImageOperator imageOperator();
  imageOperator.setCameraFrame(camera_frame);
  imageOperator.setWorldFrame(world_frame);
  imageOperator.imageToWorld(image_point, world_point_border);
  // transform point back to camera optical frame
  geometry_msgs::PointStamped test_point;
  test_point.point.x = world_point_border.x;
  test_point.point.y = world_point_border.y;
  test_point.point.z = 0.0;
  test_point.header.frame_id = "/rear_axis_middle_ground";
  geometry_msgs::PointStamped backtransformed_point;
  tf_listener_.transformPoint("/camera_optical",test_point,backtransformed_point);
  // transform the resulting point back to the image
  cv::Point3f world_test_point;
  world_test_point.x = backtransformed_point.point.x;
  world_test_point.y = backtransformed_point.point.y;
  world_test_point.z = backtransformed_point.point.z;
  cv::Point imaging_point_back;
  image_operator_.worldToImage(world_test_point, imaging_point_back);

  EXPECT_NEAR(image_point.x,imaging_point_back.x,err);
  EXPECT_NEAR(image_point.y,imaging_point_back.y,err);
}
