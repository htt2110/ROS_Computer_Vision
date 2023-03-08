#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class Sobelb(object):

    def __init__(self):

        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()

    def camera_callback(self, data):

        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e) 

        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        img = cv2.resize(img, (450,350))

        kernel_x = np.array([[-1,-2,-1],[0,0,0],[1,2,1]])
        kernel_y = np.array([[-1,0,1],[-2,-0,2],[-1,0,1]])

        x_conv = cv2.filter2D(img,-1,kernel_x)
        y_conv = cv2.filter2D(img,-1,kernel_y)

        cv2.imshow('Original', img)
        cv2.imshow('Sobelx', x_conv)
        cv2.imshow('Sobely', y_conv)

        cv2.waitKey(1)


def main():
    sobelb_object = Sobelb()
    rospy.init_node('sobelb_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
