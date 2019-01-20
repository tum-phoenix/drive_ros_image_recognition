#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class ImageProcessingNode():
    def __init__(self):
        rospy.init_node('image_processing_node_python')
        rospy.on_shutdown(self.cleanup)

        self.cv_window_name = 'Incoming image'
        cv2.namedWindow(self.cv_window_name, cv2.WINDOW_NORMAL)

        self.bridge = CvBridge()
        # Subscribe to the camera image and depth topics and set
        # the appropriate callbacks
        self.image_sub = rospy.Subscriber("~img_in", Image, self.image_callback)

    def image_callback(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image)
        except CvBridgeError, e:
            print e

        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        frame = np.array(frame, dtype=np.uint8)

        cv2.imshow(self.cv_window_name, frame)
        cv.WaitKey(1)

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        ImageProcessingNode()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down image processing node."
        cv.DestroyAllWindows()
