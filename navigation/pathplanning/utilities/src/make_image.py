import rospy
import numpy as np
import cv2 as cv2

print("Hello world")


img = np.zeros([500,500])



img[:,3] = np.ones([500])*255

print(type(img))

cv2.imwrite("test.jpg", img)

cv2.imshow('hei', img)
cv2.waitKey(0)