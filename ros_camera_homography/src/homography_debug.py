#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
import rospkg
class ImageHomographyNode:
    def __init__(self):
        rospy.init_node('image_homography_node', anonymous=True)
        self.bridge = CvBridge()
        self.left_image = None
        self.right_image = None
        self.opacity = 1.0
        self.left_points = [(50, 50), (350, 50), (350, 350), (50, 350)]
        self.right_points = [(50, 50), (350, 50), (350, 350), (50, 350)]
        self.selected_point = None
        self.dragging = False
        self.colors = [
            (0, 0, 255),
            (0, 255, 0),
            (255, 0, 0),
            (255, 255, 0)
        ]

        self.left_homography_publisher = rospy.Publisher('homography_left', Float32MultiArray, queue_size=1)
        self.right_homography_publisher = rospy.Publisher('homography_right', Float32MultiArray, queue_size=1)
        self.homography_file = rospy.get_param('~homography_file', 'homography')
        self.homography_ext = rospy.get_param('~homography_ext', 'txt')
        self.is_flip_right = rospy.get_param('~is_flip_right', False)
        
        
        self.left_subscription = rospy.Subscriber('image_left', Image, self.left_image_callback)
        cv2.namedWindow("Left Image")

    def left_image_callback(self, data):
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        cv2.imshow("Left Image", self.left_image)
        # self.update_images()

def main():
    try:
        image_homography_node = ImageHomographyNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
