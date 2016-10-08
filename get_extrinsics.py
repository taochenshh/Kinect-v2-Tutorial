#!/usr/bin/env python


import rospy
import cv2

import os
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
delta_x = 0   #0.0824
delta_y = 0   #0.064
x_num = 6
y_num = 8
objpoints = np.zeros((x_num * y_num, 3), np.float32)

class camera:

    def __init__(self):

        self.br = CvBridge()
        self.ir_image_sub = rospy.Subscriber("/kinect2/sd/image_ir",Image,self.ir_callback)
        # self.rgb_image_sub = rospy.Subscriber("/kinect2/hd/image_color",Image,self.rgb_callback)
        self.ir_img = None
        self.rgb_img = None

        self.rgb_rmat = None
        self.rgb_tvec = None
        self.ir_rmat = None
        self.ir_tvec = None

    def ir_callback(self,data):
    	try:
            ir_img = self.br.imgmsg_to_cv2(data, "passthrough")
            ir_min = np.amin(ir_img)
            ir_max = np.amax(ir_img)

            factor = 255.0 / float(ir_max - ir_min)
            self.ir_img = np.minimum(np.maximum(ir_img - ir_min, 0) * factor, 255.0)
            self.ir_img = self.ir_img.astype(np.uint8)

            clahe = cv2.createCLAHE()
            self.ir_img = clahe.apply(self.ir_img)


        except CvBridgeError as e:
            print(e)
        
        # print "ir image size: ",self.ir_img.shape

        ir_ret, ir_corners = cv2.findChessboardCorners(self.ir_img, (x_num,y_num))
        cv2.namedWindow('ir_img', cv2.WINDOW_NORMAL)
        cv2.imshow('ir_img',self.ir_img)
        cv2.waitKey(5)
        if ir_ret == True:
            ir_tempimg = self.ir_img.copy()
            cv2.cornerSubPix(ir_tempimg,ir_corners,(11,11),(-1,-1),criteria)            
            cv2.drawChessboardCorners(ir_tempimg, (x_num,y_num), ir_corners,ir_ret)
            # ret, rvec, tvec = cv2.solvePnP(objpoints, corners, mtx, dist, flags = cv2.CV_EPNP)
            ir_rvec, self.ir_tvec, ir_inliers = cv2.solvePnPRansac(objpoints, ir_corners, depth_mtx, depth_dist)
            self.ir_rmat, _ = cv2.Rodrigues(ir_rvec)

            print("The world coordinate system's origin in camera's coordinate system:")
            print("===ir_camera rvec:")
            print(ir_rvec)
            print("===ir_camera rmat:")
            print(self.ir_rmat)
            print("===ir_camera tvec:")
            print(self.ir_tvec)
            print("ir_camera_shape: ")
            print(self.ir_img.shape)

            print("The camera origin in world coordinate system:")
            print("===camera rmat:")
            print(self.ir_rmat.T)
            print("===camera tvec:")
            print(-np.dot(self.ir_rmat.T, self.ir_tvec))

            directory = '/home/chentao/kinect2_calibration'
            if not os.path.exists(directory):
                os.makedirs(directory)
            depth_stream = open("/home/chentao/kinect2_calibration/ir_camera_pose.yaml", "w")
            data = {'rmat':self.ir_rmat.tolist(), 'tvec':self.ir_tvec.tolist()}
            yaml.dump(data, depth_stream)

            
            cv2.imshow('ir_img',ir_tempimg)
            cv2.waitKey(5)

    def rgb_callback(self,data):
        try:
        	self.rgb_img = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        gray = cv2.cvtColor(self.rgb_img,cv2.COLOR_BGR2GRAY)
        rgb_ret, rgb_corners = cv2.findChessboardCorners(gray, (x_num,y_num),None)
        cv2.namedWindow('rgb_img', cv2.WINDOW_NORMAL)
        cv2.imshow('rgb_img',self.rgb_img)
        cv2.waitKey(5)
        if rgb_ret == True:
            rgb_tempimg = self.rgb_img.copy()
            cv2.cornerSubPix(gray,rgb_corners,(5,5),(-1,-1),criteria)            
            cv2.drawChessboardCorners(rgb_tempimg, (x_num,y_num), rgb_corners,rgb_ret)
            rgb_rvec, self.rgb_tvec, rgb_inliers = cv2.solvePnPRansac(objpoints, rgb_corners, rgb_mtx, rgb_dist)
            self.rgb_rmat, _ = cv2.Rodrigues(rgb_rvec)
            print("The world coordinate system's origin in camera's coordinate system:")
            print("===rgb_camera rvec:")
            print(rgb_rvec)
            print("===rgb_camera rmat:")
            print(self.rgb_rmat)
            print("===rgb_camera tvec:")
            print(self.rgb_tvec)
            print("rgb_camera_shape: ")
            print(self.rgb_img.shape)

            print("The camera origin in world coordinate system:")
            print("===camera rmat:")
            print(self.rgb_rmat.T)
            print("===camera tvec:")
            print(-np.dot(self.rgb_rmat.T, self.rgb_tvec))

            directory = '/home/chentao/kinect2_calibration'
            if not os.path.exists(directory):
                os.makedirs(directory)

            rgb_stream = open("/home/chentao/kinect2_calibration/rgb_camera_pose.yaml", "w")
            data = {'rmat':self.rgb_rmat.tolist(), 'tvec':self.rgb_tvec.tolist()}
            yaml.dump(data, rgb_stream)

            
            cv2.imshow('rgb_img',rgb_tempimg)
            cv2.waitKey(5)

    def mkgray(self, msg):
        """
        Convert a message into a 8-bit 1 channel monochrome OpenCV image
        """
        # as cv_bridge automatically scales, we need to remove that behavior
        # TODO: get a Python API in cv_bridge to check for the image depth.
        if self.br.encoding_to_dtype_with_channels(msg.encoding)[0] in ['uint16', 'int16']:
            mono16 = self.br.imgmsg_to_cv2(msg, '16UC1')
            mono8 = np.array(np.clip(mono16, 0, 255), dtype=np.uint8)
            return mono8
        elif 'FC1' in msg.encoding:
            # floating point image handling
            img = self.br.imgmsg_to_cv2(msg, "passthrough")
            _, max_val, _, _ = cv2.minMaxLoc(img)
            if max_val > 0:
                scale = 255.0 / max_val
                mono_img = (img * scale).astype(np.uint8)
            else:
                mono_img = img.astype(np.uint8)
            return mono_img
        else:
        	return self.br.imgmsg_to_cv2(msg, "mono8")

if __name__ == "__main__":
    
    for i in range(y_num):
        for j in range(x_num):
            index = i * x_num + j
            objpoints[index,0] = delta_x + j * width
            objpoints[index,1] = delta_y + (y_num - 1 - i) * width
            objpoints[index,2] = 0
    rospy.init_node('get_extrinsics')
    ic = camera()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
