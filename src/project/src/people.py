#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist


class LoadPeople(object):

    def __init__(self):

        self.image_sub = rospy.Subscriber("camera/rgb/img_raw", Image, self.camera_callback)    
        self.bridge_object = CvBridge()
        self.a = 0.0
        self.b = 0.0

    def sonar_callback(self, msg):
        
        self.a = msg.range

    def camera_callback(self, data):

        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        imX = 700
        imY = 500

        img_2 = cv2.resize(cv_image,(imX, imY))

        gray_2 = cv2.cvtColor(img_2, cv2.COLOR_BGR2GRAY)

        boxes_2, weights_2 = hog.detectMultiScale(gray_2, winStride=(8,6))
        boxes_2 = np.array([[x,y,x+w,y+h] for (x,y,w,h) in boxes_2])

        for (xA,yA,xB,yB) in boxes_2:

            medX = xB - xA 
            xC = int(xA+(medX/2))

            medY = yB - yA 
            yC = int(yA+(medY/2))

            cv2.circle(img_2,(xC,yC),1,(0,255,255),-1)

            cv2.rectangle(img_2, (xA,yA), (xB,yB), (255,255,0), 2)
        
        cv2.imshow('image', img_2)

        cv2.waitkey(1)

def main():
    load_people_object = LoadPeople()
    rospy.init_node('load_people_node', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()