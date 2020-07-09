#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
import imutils
import time,math
import numpy as np
import yaml,os,sys

class StructurePointxnynanRead():
    def __init__(self,z_dstar,a_dstar):
        self.bridge = CvBridge()
        self.rgb_image=None
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.cross_xn_pub = rospy.Publisher("/cross_line_xsubn", Float64, queue_size=10)
        self.cross_yn_pub = rospy.Publisher("/cross_line_ysubn", Float64, queue_size=10)
        self.cross_an_pub = rospy.Publisher("/cross_line_asubn", Float64, queue_size=10)
        self.cross_area_pub = rospy.Publisher("/cross_line_area", Float64, queue_size=10)

        self.z_dstar = z_dstar
        self.a_dstar = a_dstar        
        self.califilename="/home/zy/catkin_ws/src/polishingrobot_lx/visionbased_polishing/yaml/cam_300_industry_20200518.yaml"
        self.file=open(self.califilename)
        self.yamldata=yaml.load(self.file)
        self.f = 0.6245768 # self.yamldata['focal_length']
        self.kx = self.yamldata['camera_matrix']['data'][0]
        self.ky = self.yamldata['camera_matrix']['data'][4]
        self.u0 = self.yamldata['camera_matrix']['data'][2]
        self.v0 = self.yamldata['camera_matrix']['data'][5]
        self.pu=self.f/self.kx
        self.pv=self.f/self.ky
        self.centra_uv=[self.u0,self.v0]
        self.flag=0

    def callback(self,data):
        try:
            video_capture=self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.rgb_image = video_capture.copy()
        except CvBridgeError as e:
            print(e)

    "calculate Euler distance"
    def cal_distance(self,point1, point2):
        dis = np.sqrt(np.sum(np.square(point1[0] - point2[0]) + np.square(point1[1] - point2[1])))
        return dis

    "calculate area based on helen formula"
    def helen_formula(self,coord):
        coord = np.array(coord).reshape((4, 2))
        dis_01 = self.cal_distance(coord[0], coord[1])
        dis_12 = self.cal_distance(coord[1], coord[2])
        dis_23 = self.cal_distance(coord[2], coord[3])
        dis_13 = self.cal_distance(coord[1], coord[3])
        dis_30 = self.cal_distance(coord[3], coord[0])
        p1 = (dis_01 + dis_30 + dis_13) * 0.5
        p2 = (dis_12 + dis_23 + dis_13) * 0.5
        area1 = np.sqrt(p1 * (p1 - dis_01) * (p1 - dis_30) * (p1 - dis_13))
        area2 = np.sqrt(p2 * (p2 - dis_12) * (p2 - dis_23) * (p2 - dis_13))
        return (area1 + area2) / 2

    def get_xnynzn(self,centroid_point,area_inrealtime):
        z_inrealtime=self.z_dstar*math.sqrt(self.a_dstar/area_inrealtime)
        xg,yg=self.change_uv_to_cartisian(centroid_point)
        x_inrealtime=xg*z_inrealtime
        y_inrealtime=yg*z_inrealtime
        return x_inrealtime,y_inrealtime,z_inrealtime

    def change_uv_to_cartisian(self,point1):
        x=(point1[0]-self.centra_uv[0])*(self.pu/self.f)
        y=(point1[1]-self.centra_uv[1])*(self.pv/self.f)
        return x,y
    
    def process_rgb_image(self,image):
        if image is not None:
            "image processing"
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            kernel1 = cv2.getStructuringElement(cv2.MORPH_RECT,(5, 5))
            eroded1 = cv2.erode(gray,kernel1)
            dilated1 = cv2.dilate(eroded1,kernel1)
            dilated1 = cv2.dilate(dilated1,kernel1)
            eroded1 = cv2.erode(dilated1,kernel1)
            blurred = cv2.GaussianBlur(gray, (11,11), 0)

            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # 将读取的BGR转换为HSV
            lower = np.array([90, 43, 20])  # 所要检测的像素范围
            upper = np.array([150, 255, 255])  # 此处检测绿色区域
            mask = cv2.inRange(hsv, lowerb=lower, upperb=upper)
            cv2.imshow("mask", mask)

            # thresh = cv2.threshold(blurred, 205, 255, cv2.THRESH_BINARY)[1]
            # thresh = cv2.threshold(blurred, 190, 255, cv2.THRESH_BINARY)[1]
            # cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            c = max(cnts, key=cv2.contourArea)

            "publish xn yn zn an"
            extLeft = tuple(c[c[:, :, 0].argmin()][0])
            extRight = tuple(c[c[:, :, 0].argmax()][0])
            extTop = tuple(c[c[:, :, 1].argmin()][0])
            extBot = tuple(c[c[:, :, 1].argmax()][0])
            font = cv2.FONT_HERSHEY_SIMPLEX
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            now_central = (cX, cY)
            
            x0,y0=self.change_uv_to_cartisian(extLeft)
            x1,y1=self.change_uv_to_cartisian(extTop)
            x2,y2=self.change_uv_to_cartisian(extRight)
            x3,y3=self.change_uv_to_cartisian(extBot)
            area = self.helen_formula([x0,y0, x1,y1, x2,y2, x3,y3])
            x_inrealtime,y_inrealtime,z_inrealtime=self.get_xnynzn(now_central,area)
            
            self.cross_xn_pub.publish(x_inrealtime)
            self.cross_yn_pub.publish(y_inrealtime)
            self.cross_an_pub.publish(z_inrealtime)
            self.cross_area_pub.publish(area)
            print "real x,y,z,area is:",x_inrealtime,y_inrealtime,z_inrealtime, area
            # print("real uv info is:",now_central)
            rospy.logerr("real uv info is: %s"%str(now_central))

            "show the windows"
            cv2.line(image, extLeft, extTop, [0, 255, 0], 2)
            cv2.line(image, extLeft, extBot, [0, 255, 0], 2)
            cv2.line(image, extRight, extTop, [0, 255, 0], 2)
            cv2.line(image, extRight, extBot, [0, 255, 0], 2)
            cv2.drawContours(image, [c], -1, (0, 255, 255), 2)
            cv2.circle(image, extLeft, 10, (0, 0, 255), -1)
            cv2.circle(image, extRight, 10, (0, 0, 255), -1)
            cv2.circle(image, extTop, 10, (0, 0, 255), -1)
            cv2.circle(image, extBot, 10, (0, 0, 255), -1)
            cv2.circle(image, now_central, 10, (0, 0, 255), -1)
            cv2.namedWindow('central_frame', cv2.WINDOW_NORMAL)
            cv2.imshow("central_frame", image)
            cv2.waitKey(8)

def main():
    try:
        rospy.init_node("Generation_image_feature")
        rospy.loginfo("Starting generation image features node")
        flag=0
        z_dstar=0.20
        a_dstar=0.0050
        Generation_image_feature=StructurePointxnynanRead(z_dstar,a_dstar)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            Generation_image_feature.process_rgb_image(Generation_image_feature.rgb_image)
            rate.sleep()
    except KeyboardInterrupt:
        print "Stopping generation image features node"
        cv2.destroyAllWindows()
if __name__=="__main__":
    main()
