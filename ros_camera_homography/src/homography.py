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
        self.left_subscription = rospy.Subscriber('image_left', Image, self.left_image_callback)
        self.right_subscription = rospy.Subscriber('image_right', Image, self.right_image_callback)

        self.homography_matrix = None
        rospack = rospkg.RosPack()
        self.save_dir = os.path.join(rospack.get_path('ros_camera_homography'),'homography_files')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        self.save_homography_path = os.path.join(self.save_dir,
                                                 self.homography_file)  

        cv2.namedWindow("Left Image")
        cv2.namedWindow("Right Image")
        cv2.namedWindow("Homography Result")
        cv2.namedWindow("Controls", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Controls", 400, 100)
        cv2.createTrackbar('Opacity', 'Controls', 100, 100, self.on_trackbar)
        cv2.setMouseCallback("Left Image", self.handle_mouse_events, "left")
        cv2.setMouseCallback("Right Image", self.handle_mouse_events, "right")
        cv2.setMouseCallback("Controls", self.handle_mouse_events, "save")
        self.button = np.zeros((50, 100, 3), dtype=np.uint8)
        cv2.putText(self.button, 'Save', (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        self.button_clicked = False

    def on_trackbar(self, val):
        self.opacity = val / 100.0

    def handle_mouse_events(self, event, x, y, flags, param):
        if param in ["left", "right"]:
            points = self.left_points if param == "left" else self.right_points
            if event == cv2.EVENT_LBUTTONDOWN:
                for i, point in enumerate(points):
                    if abs(x - point[0]) < 10 and abs(y - point[1]) < 10:
                        self.selected_point = (i, param)
                        self.dragging = True
                        break
            if event == cv2.EVENT_LBUTTONUP and self.dragging:
                self.selected_point = None
                self.dragging = False
            if event == cv2.EVENT_MOUSEMOVE and self.dragging:
                idx, side = self.selected_point
                if side == "left":
                    self.left_points[idx] = (x, y)
                else:
                    self.right_points[idx] = (x, y)
        elif param in ["save"]:
            if event == cv2.EVENT_LBUTTONDOWN:
                if 0 < x < 100 and 0 < y < 50:
                    self.save_homography_matrix()
                    
    def left_image_callback(self, data):
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        self.update_images()

    def right_image_callback(self, data):
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        self.update_images()

    def update_images(self):
        if self.left_image is not None:
            left_image_with_points = self.left_image.copy()
            for i in range(len(self.left_points)):
                cv2.circle(left_image_with_points, self.left_points[i], 5, self.colors[i], -1)
            cv2.imshow("Left Image", left_image_with_points)

        if self.right_image is not None:
            right_image_with_points = self.right_image.copy()
            for i in range(len(self.right_points)):
                cv2.circle(right_image_with_points, self.right_points[i], 5, self.colors[i], -1)
            cv2.imshow("Right Image", right_image_with_points)

        if self.left_image is not None and self.right_image is not None:
            self.compute_homography()
        cv2.imshow("Controls", self.button)
        cv2.waitKey(1)

    def compute_homography(self):
        H, _ = cv2.findHomography(np.array(self.right_points), np.array(self.left_points))
        height, width, _ = self.left_image.shape
        self.homography_matrix = H

        warped_right = cv2.warpPerspective(self.right_image, H, (width, height))
        overlap_area = cv2.addWeighted(self.left_image, 1 - self.opacity, warped_right, self.opacity, 0)
        cv2.imshow("Homography Result", overlap_area)
        cv2.waitKey(1)

    def save_homography_matrix(self):
        if self.homography_matrix is not None:
            
            if self.homography_ext == 'npy':
                np.save(f'{self.save_homography_path}.npy', self.homography_matrix)
                rospy.loginfo(f"Homography matrix saved to {self.save_homography_path}.npy")
            elif self.homography_ext == 'txt':
                np.savetxt(f'{self.save_homography_path}.txt', self.homography_matrix)
                rospy.loginfo(f"Homography matrix saved to {self.save_homography_path}.txt")
            msg = Float32MultiArray()
            msg.data = self.homography_matrix.flatten().tolist()
            self.right_homography_publisher.publish(msg)

            identity_matrix = np.identity(3)
            msg = Float32MultiArray()
            msg.data = identity_matrix.flatten().tolist()
            self.left_homography_publisher.publish(msg)  

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
