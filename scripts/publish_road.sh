# simple publisher for road messages to be able to view what happens during processing
rostopic pub /road_trajectory drive_ros_msgs/RoadLane "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
points:
- x: 0.0
  y: 0.0
  z: 0.0
- x: 1.0
  y: 0.0
  z: 0.0
roadStateType: 0" -r 100
