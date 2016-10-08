#!/usr/bin/env python


import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import yaml
import sys
from matplotlib import pyplot as plt

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

skip_lines = 1
with open("/home/chentao/catkin_ws/src/iai_kinect2/kinect2_bridge/data/035623243247/calib_ir.yaml") as depth_stream:
    for i in range(skip_lines):
        _ = depth_stream.readline()

    depth_doc = yaml.load(depth_stream)
    depth_mtx = np.array(depth_doc['cameraMatrix']['data']).reshape(3,3)
    depth_dist = np.array(depth_doc['distortionCoefficients']['data'])


with open("/home/chentao/catkin_ws/src/iai_kinect2/kinect2_bridge/data/035623243247/calib_color.yaml") as rgb_stream:
    for i in range(skip_lines):
        _ = rgb_stream.readline()
    rgb_doc = yaml.load(rgb_stream)
    rgb_mtx = np.array(rgb_doc['cameraMatrix']['data']).reshape(3,3)
    rgb_dist = np.array(rgb_doc['distortionCoefficients']['data'])

width = 0.0348
delta_x = 0    #0.0824
delta_y = 0    #0.064
x_num = 6
y_num = 8
objpoints = np.zeros((x_num * y_num, 3), np.float32)

axis = np.float32([[delta_x,delta_y,0],[delta_x + width,delta_y,0], [delta_x,delta_y + width,0], [delta_x,delta_y,delta_y + width]]).reshape(-1,3)
axis[:,0] += width * 3
axis[:,1] += width * 3

class image_converter:

    def __init__(self):

        self.br = CvBridge()
        # self.image_sub = rospy.Subscriber("/kinect2/hd/image_color",Image,self.rgb_callback)
        self.image_sub = rospy.Subscriber("/kinect2/sd/image_ir",Image,self.ir_callback)

    def rgb_callback(self,data):
        try:
            img = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # ret, corners = cv2.findChessboardCorners(gray, (x_num,y_num),None)
        ret, corners = cv2.findChessboardCorners(img, (x_num,y_num))
        cv2.imshow('img',img)
        cv2.waitKey(5)
        if ret == True:
            cv2.cornerSubPix(gray,corners,(5,5),(-1,-1),criteria)
            tempimg = img.copy()
            cv2.drawChessboardCorners(tempimg, (x_num,y_num), corners,ret)
            # ret, rvec, tvec = cv2.solvePnP(objpoints, corners, mtx, dist, flags = cv2.CV_EPNP)
            rvec, tvec, inliers = cv2.solvePnPRansac(objpoints, corners, rgb_mtx, rgb_dist)
            print("rvecs:")
            print(rvec)
            print("tvecs:")
            print(tvec)
            # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(axis, rvec, tvec, rgb_mtx, rgb_dist)

            imgpts = np.int32(imgpts).reshape(-1,2)

            cv2.line(tempimg, tuple(imgpts[0]), tuple(imgpts[1]),[255,0,0],4)  #BGR
            cv2.line(tempimg, tuple(imgpts[0]), tuple(imgpts[2]),[0,255,0],4)
            cv2.line(tempimg, tuple(imgpts[0]), tuple(imgpts[3]),[0,0,255],4)

            cv2.imshow('img',tempimg)
            cv2.waitKey(5)

    def ir_callback(self,data):
        try:
            ir_img = self.br.imgmsg_to_cv2(data, "passthrough")
            ir_min = np.amin(ir_img)
            ir_max = np.amax(ir_img)

            factor = 255.0 / float(ir_max - ir_min)
            ir_img = np.minimum(np.maximum(ir_img - ir_min, 0) * factor, 255.0)
            ir_img = ir_img.astype(np.uint8)

            clahe = cv2.createCLAHE()
            gray = clahe.apply(ir_img)


        except CvBridgeError as e:
            print(e)

        # ret, corners = cv2.findChessboardCorners(gray, (x_num,y_num),None)
        ret, corners = cv2.findChessboardCorners(gray, (x_num,y_num))
        cv2.imshow('img',gray)
        cv2.waitKey(5)
        if ret == True:
            cv2.cornerSubPix(gray,corners,(5,5),(-1,-1),criteria)
            tempimg = gray.copy()
            cv2.drawChessboardCorners(tempimg, (x_num,y_num), corners,ret)
            # ret, rvec, tvec = cv2.solvePnP(objpoints, corners, mtx, dist, flags = cv2.CV_EPNP)
            rvec, tvec, inliers = cv2.solvePnPRansac(objpoints, corners, depth_mtx, depth_dist)
            print("rvecs:")
            print(rvec)
            print("tvecs:")
            print(tvec)
            # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(axis, rvec, tvec, depth_mtx, depth_dist)

            imgpts = np.int32(imgpts).reshape(-1,2)

            cv2.line(tempimg, tuple(imgpts[0]), tuple(imgpts[1]),[255,0,0],4)  #BGR
            cv2.line(tempimg, tuple(imgpts[0]), tuple(imgpts[2]),[0,255,0],4)
            cv2.line(tempimg, tuple(imgpts[0]), tuple(imgpts[3]),[0,0,255],4)

            cv2.imshow('img',tempimg)
            cv2.waitKey(5)


if __name__ == "__main__":
    ic = image_converter()
    for i in range(y_num):
        for j in range(x_num):
            index = i * x_num + j
            objpoints[index,0] = delta_x + j * width
            objpoints[index,1] = delta_y + (y_num - 1 - i) * width
            objpoints[index,2] = 0
    print objpoints
    rospy.init_node('pose_estimation')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
