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
"""
use structure line for detecting
"""
class DetectSturctureLine:
    def __init__(self,z_dstar,a_dstar):
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.rgb_image=None
        self.cross_xn_pub = rospy.Publisher("/cross_line_xsubn", Float64, queue_size=10)
        self.cross_yn_pub = rospy.Publisher("/cross_line_ysubn", Float64, queue_size=10)
        self.cross_an_pub = rospy.Publisher("/cross_line_asubn", Float64, queue_size=10)
        self.cross_area_pub = rospy.Publisher("/cross_line_area", Float64, queue_size=10)
        self.resuv=[]
        self.z_dstar=z_dstar
        self.a_dstar = a_dstar
        self.f=0.6245768
        self.pu=self.f/624.576 #476.437121#f/kx
        self.pv=self.f/625.9805
        self.centra_uv=[305,255]
        self.flag=0
    def callback(self,data):
        try:
            video_capture=self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.rgb_image = video_capture.copy()
        except CvBridgeError as e:
            print(e)

    # 计算欧式距离
    def cal_distance(self,point1, point2):
        dis = np.sqrt(np.sum(np.square(point1[0] - point2[0]) + np.square(point1[1] - point2[1])))
        return dis

    # 基于海伦公式计算不规则四边形的面积
    def helen_formula(self,coord):
        coord = np.array(coord).reshape((4, 2))
        # 计算各边的欧式距离
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

    # 基于向量积计算不规则四边形的面积
    def vector_product(self,coord):
        coord = np.array(coord).reshape((4, 2))
        temp_det = 0
        for idx in range(3):
            temp = np.array([coord[idx], coord[idx + 1]])
            temp_det += np.linalg.det(temp)
        temp_det += np.linalg.det(np.array([coord[-1], coord[0]]))
        return temp_det / 4

    def get_xnynzn(self,centroid_point,area_inrealtime):
        asubn=self.z_dstar*math.sqrt(self.a_dstar/area_inrealtime)
        xg,yg=self.change_uv_to_cartisian(centroid_point)
        xsubn=asubn*xg
        ysubn=asubn*yg
        return [xsubn,ysubn,asubn]

    def change_uv_to_cartisian(self,point1):
        x=(point1[0]-self.centra_uv[0])*(self.pu/self.f)
        y=(point1[1]-self.centra_uv[1])*(self.pv/self.f)
        return x,y

    def process_rgb_image(self,rgb_image):
        image = rgb_image
        if image is not None:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            kernel1 = cv2.getStructuringElement(cv2.MORPH_RECT,(5, 5))
            eroded1 = cv2.erode(gray,kernel1)
            dilated1 = cv2.dilate(eroded1,kernel1)
            dilated1 = cv2.dilate(dilated1,kernel1)
            eroded1 = cv2.erode(dilated1,kernel1)
            blurred = cv2.GaussianBlur(gray, (11,11), 0)
            thresh = cv2.threshold(blurred, 205, 255, cv2.THRESH_BINARY)[1]
            cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            # print(cnts)
            
            cnts = imutils.grab_contours(cnts)
            c = max(cnts, key=cv2.contourArea)

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
            cv2.line(image, extLeft, extTop, [0, 255, 0], 2)
            cv2.line(image, extLeft, extBot, [0, 255, 0], 2)
            cv2.line(image, extRight, extTop, [0, 255, 0], 2)
            cv2.line(image, extRight, extBot, [0, 255, 0], 2)

            print("the central points is located at:",now_central)
            xn,yn,an=self.get_xnynzn(now_central,area)
            xg, yg = self.change_uv_to_cartisian(now_central)
            if self.flag>50:
                if an <=self.z_dstar:
                    print "an <=self.z_dstar",an <=self.z_dstar
                    ann=self.z_dstar-(an*100-int(an*100))/100
                    self.cross_xn_pub.publish(xg*ann)
                    self.cross_yn_pub.publish(yg*ann)
                    self.cross_an_pub.publish(ann)
                else :
                    ann = self.z_dstar - (an * 100 - int(an * 100)) / 100
                    # print "self.z_dstar - (an * 100 - int(an * 100)) / 100",self.z_dstar - (an * 100 - int(an * 100)) / 100
                    self.cross_xn_pub.publish(xg*ann)
                    self.cross_yn_pub.publish(yg*ann)
                    self.cross_an_pub.publish(ann)
            else:
                self.cross_xn_pub.publish(xn)
                self.cross_yn_pub.publish(yn)
                self.cross_an_pub.publish(an)
            self.flag+=1
            print "self.flag",self.flag
            if self.flag>55:
                self.flag=55
            self.cross_area_pub.publish(area)
            print "xn,yn,an",xn,yn,an
            cv2.drawContours(image, [c], -1, (0, 255, 255), 2)

            cv2.circle(image, extLeft, 10, (0, 0, 255), -1)
            cv2.circle(image, extRight, 10, (0, 0, 255), -1)
            cv2.circle(image, extTop, 10, (0, 0, 255), -1)
            cv2.circle(image, extBot, 10, (0, 0, 255), -1)
            cv2.circle(image, now_central, 10, (0, 0, 255), -1)

            cv2.namedWindow('central_frame_3', cv2.WINDOW_NORMAL)
            cv2.imshow('central_frame_3', thresh)

            cv2.namedWindow('central_frame', cv2.WINDOW_NORMAL)
            cv2.imshow("central_frame", image)
            cv2.waitKey(8)
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(rgb_image, "bgr8"))
            except CvBridgeError as e:
                print e
    def papers_alog(self,rgb_image):
        self.process_rgb_image(rgb_image)

def main():
    try:
        z_dstar=0.22#0.31
        a_dstar=0.00616# 0.00997#0.0148#0.00987#0.00678718
        flag=0
        rospy.init_node("Detect_Sturcture_Line")
        rospy.loginfo("Starting DetectSturctureLine node")
        k=DetectSturctureLine(z_dstar,a_dstar)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            cen=k.process_rgb_image(k.rgb_image)
            rate.sleep()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node."
        cv2.destroyAllWindows()
if __name__=="__main__":
    main()
