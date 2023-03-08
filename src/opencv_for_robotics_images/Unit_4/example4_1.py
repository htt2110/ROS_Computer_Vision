#! /usr/bin/env python

import cv2
import numpy as np

image = cv2.imread('/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_4/Course_images/corner_test_2.png')
gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

fast = cv2.FastFeatureDetector_create()

Keypoints1 = fast.detect(gray, None)
fast.setNonmaxSuppression(False)

Keypoints2 = fast.detect(gray, None)

image_without_nonmax = np.copy(image)

cv2.drawKeypoints(image, Keypoints2, image_without_nonmax, color=(0,35,250))

cv2.imshow('Without non max Supression',image_without_nonmax)
cv2.waitKey(0)
cv2.destroyAllWindows()
