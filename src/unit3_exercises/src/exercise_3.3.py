#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class LoadMultipleFace(object):

    def __init__(self):

        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()

    def camera_callback(self, data):

        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        face_cascade = cv2.CascadeClassifier('/home/user/catkin_ws/src/unit3_exercises/haar_cascades/frontalface.xml')
        
        

        
        img_original = cv2.imread('/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_3/Course_images/many.jpg')
        

        img_original = cv2.resize(img_original,(700,600))

        img = cv2.resize(img_original,(700,600))
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        ScaleFactor = 1.2
        minNeighbors = 3

        faces = face_cascade.detectMultiScale(gray, ScaleFactor, minNeighbors)
        
        for (x,y,w,h) in faces:
            
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,255,0),2)  
            roi = img[y:y+h, x:x+w]
    

        cv2.imshow('Faces_original',img_original)
            
        cv2.imshow('Faces',img)
        
                
        cv2.waitKey(1)



def main():
    load_faces_object = LoadMultipleFace()
    rospy.init_node('load_faces_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
