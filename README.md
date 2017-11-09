# drive_ros_image_recognition
image recognition (lanes, crossings, start lines, ....)

For using the line detection, we need
* a camera image (from ximea camera or a rosbag)
* the homography. Therefore you can use `drive_ros_ximea_importer/launch/homography_publischer_only.launch`.

Now start `drive_ros_image_recognition/launch/line_recognition.launch`.
For `drive_ros_image_recognition/launch/warp_image.launch` you have to do the same two steps from above.
