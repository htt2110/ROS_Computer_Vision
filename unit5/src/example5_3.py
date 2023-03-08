#! /usr/bin/env python

import cv2
import numpy as np
from cv2 import aruco

def order_coordinates(pts):

    coordinates = np.zeros((4,2), dtype="int")

    s = pts.sum(axis=1)
    coordinates[0] = pts[np.argmin(s)]
    coordinates[2] = pts[np.argmax(s)]

    diff = np.diff(pts, axis=1)
    coordinates[1] = pts[np.argmin(diff)]
    coordinates[3] = pts[np.argmax(diff)]

    return coordinates

image = cv2.imread('/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_5/Course_images/Examples/a3.jpg')
h,w = image.shape[:2]

image = cv2.resize(image, (int(w*0.7),int(h*0.7)))
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters_create()

corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)

cv2.imshow('Markers', frame_markers)

params = []

for i in range(len(ids)):

    c =corners[i][0]

    cv2.circle(image,(int(c[:,0].mean()),int(c[:,1].mean())), 3, (255,255,0), -1)

    params.append((int(c[:,0].mean()), int(c[:,1].mean())))

params = np.array(params)

cv2.drawContours(image, [params],-1,(255,0,150),-1)

cv2.imshow('No_Conversion', image)
cv2.waitKey(0)
cv2.destroyAllWindows()