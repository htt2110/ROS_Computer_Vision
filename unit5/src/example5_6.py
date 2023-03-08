import os
import cv2
import numpy as np
from cv2 import aruco

print(cv2.__file__)

def order_coordinates(pts, var):

    coordinates = np.zeros((4,2), dtype = "int")

    if(var):
        #Parameters for sort model 1
        s = pts.sum(axis=1)
        coordinates[0] = pts[np.argmin(s)]
        coordinates[3] = pts[np.argmax(s)]

        diff = np.diff(pts, axis=1)
        coordinates[1] = pts[np.argmin(diff)]
        coordinates[2] = pts[np.argmax(diff)]

    else:
        #Parameters for sort model 2
        s = pts.sum(axis=1)
        coordinates[0] = pts[np.argmin(s)]
        coordinates[2] = pts[np.argmax(s)]

        diff = np.diff(pts, axis=1)
        coordinates[1] = pts[np.argmin(diff)]
        coordinates[3] = pts[np.argmax(diff)]

    return coordinates

image = cv2.imread('/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_5/Course_images/Examples/a1.jpg')
h, w = image.shape[:2]

image = cv2.resize(image, (int(w*0.7), int(h*0.7))) 
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters_create()

corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

params = []

for i in range(len(ids)):

    c = corners[i][0]

    cv2.circle(image, (int(c[:,0].mean()), int(c[:,1].mean())), 3, (255,255,0), -1)

    params.append((int(c[:,0].mean()), int(c[:,1].mean())))

params = np.array(params)

if(len(params)>=4):

    params = order_coordinates(params, False)

    params_2 = order_coordinates(params,True)

paint = cv2.imread('/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_5/Course_images/Examples/earth.jpg')

height, width = paint.shape[:2]

coordinates = np.array([[0,0],[width,0],[0,height],[width,height]])

hom, status = cv2.findHomography(coordinates, params_2)

warped_image = cv2.warpPerspective(paint, hom, (int(w*0.7), int(h*0.7)))

mask = np.zeros([int(h*0.7), int(w*0.7), 3], dtype = np.uint8)

cv2.fillConvexPoly(mask, np.int32([params]), (255,255,255), cv2.LINE_AA)
cv2.imshow('black_mask', mask)
substraction = cv2.subtract(image,mask)
cv2.imshow('substraction', substraction)
addition = cv2.add(warped_image, substraction)
cv2.imshow('detection', addition)

cv2.waitKey(0)
cv2.destroyAllWindows()
