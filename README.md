# drive_ros_image_recognition
image recognition (lanes, crossings, start lines, ....)

For using the line detection, we need
* a camera image (from ximea camera or a rosbag)
* the homography. Therefore you can use `drive_ros_ximea_importer/launch/homography_publischer_only.launch`.

Now start `drive_ros_image_recognition/launch/line_recognition.launch`.

The currently used files are:
* /include/drive_ros_image_recognition/common_image_operations.h
* /include/drive_ros_image_recognition/line_detection.h
* /include/drive_ros_image_recognition/street_line.h
* /launch/line_detection.h
* /src/line_detection.cpp
* /src/line_detection_node.cpp
