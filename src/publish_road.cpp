#include <ros/ros.h>
#include <drive_ros_msgs/RoadLane.h>
#include <geometry_msgs/PointStamped.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_road");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

#ifndef NDEBUG
  // give GDB time to attach
  ros::Duration(2.0).sleep();
#endif

  ros::Publisher point_pub = nh.advertise<drive_ros_msgs::RoadLane>("/road_detection/road_in",10);
  // publish PointStamped as well so we can display in rviz
  ros::Publisher point_stamped_pub = nh.advertise<geometry_msgs::PointStamped>("road_points",10);
  ros::Rate rate(50);

  while (ros::ok()) {

    drive_ros_msgs::RoadLane my_road;
    my_road.header.stamp = ros::Time::now();
    my_road.header.frame_id = "front_axis_middle";
    my_road.points.resize(2);
    geometry_msgs::PointStamped point;
    point.header.stamp = ros::Time::now();
    point.header.frame_id = "front_axis_middle";
    point.point.x = 0.0;
    point.point.y = 0.0;
    point.point.z = 0.0;
    my_road.points[0] = point;
    point_stamped_pub.publish(point);
    point.point.x = 0.1;
    point.point.y = 0.1;
    point.point.z = 0.0;
    my_road.points[1] = point;
    point_pub.publish(my_road);
    point_stamped_pub.publish(point);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
