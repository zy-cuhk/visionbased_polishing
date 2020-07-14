#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import imutils
import time,math
import numpy as np
import yaml,os,sys
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

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

    def cal_distance(self,point1, point2):
        dis = np.sqrt(np.sum(np.square(point1[0] - point2[0]) + np.square(point1[1] - point2[1])))
        return dis

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

    def centroid_computation(self,points):
        points1=points.reshape(4,2)
        sum_x=0
        sum_y=0
        for i in range(len(points1)):
            sum_x+=points1[i,0]
            sum_y+=points1[i,1]
        cx=int(sum_x/len(points1))
        cy=int(sum_y/len(points1))
        now_central = (cx, cy)
        return now_central


    def image_process(self,img):
        # cv2.namedWindow('image', cv2.WINDOW_NORMAL)

        hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        lower_red=np.array([0,50,50])
        upper_red=np.array([10,255,255])
        mask=cv2.inRange(hsv,lower_red,upper_red)
        res=cv2.bitwise_and(img,img,mask=mask)
        mask1=mask.copy()
        cnts = cv2.findContours(mask1, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        cnts1=sorted(cnts,key=cv2.contourArea,reverse=True)
        for i in range(len(cnts1)):
            print("the area is:",cv2.contourArea(cnts1[i], True))
        
        # cv2.imshow('the captured video',img)
        cv2.imshow("the mask",mask)
        cv2.waitKey(8)
        # cv2.imshow("the res",res)

        print("the contour number is:",len(cnts1))
        if len(cnts1)>=4:
            points=[]
            for i in range(0,4):
                c=cnts1[i]
                M = cv2.moments(c)
                if M["m00"]!=0.0:
                    cx= int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    now_central = (cx, cy)
                    cv2.circle(img, now_central, 10, (0, 0, 255), -1)
                    points.append(int(cx))
                    points.append(int(cy))

                    if len(points)==8:
                        xlist=[]
                        ylist=[]
                        for i in range(len(points)/2):
                            xlist.append(int(points[2*i]))
                            ylist.append(int(points[2*i+1]))
                        xmin_index=np.array(xlist).argmin()
                        xmax_index=np.array(xlist).argmax()
                        ymin_index=np.array(ylist).argmin()
                        ymax_index=np.array(ylist).argmax()
                        points=np.array(points).reshape(4,2)
                        left_point=(points[xmin_index,0],points[xmin_index,1])
                        right_point=(points[xmax_index,0],points[xmax_index,1])
                        bot_point=(points[ymin_index,0],points[ymin_index,1])
                        top_point=(points[ymax_index,0],points[ymax_index,1])
                        cv2.line(img, left_point, top_point, [0, 255, 0], 2)
                        cv2.line(img, left_point, bot_point, [0, 255, 0], 2)
                        cv2.line(img, right_point, top_point, [0, 255, 0], 2)
                        cv2.line(img, right_point, bot_point, [0, 255, 0], 2)
                        now_central=self.centroid_computation(points)
                        cv2.circle(img, now_central, 10, (0, 0, 255), -1)
                        cv2.imshow('the captured video',img)
                        cv2.waitKey(8)


                        x0,y0=self.change_uv_to_cartisian(left_point)
                        x1,y1=self.change_uv_to_cartisian(top_point)
                        x2,y2=self.change_uv_to_cartisian(right_point)
                        x3,y3=self.change_uv_to_cartisian(bot_point)
                        area = self.helen_formula([x0,y0, x1,y1, x2,y2, x3,y3])
                        x_inrealtime,y_inrealtime,z_inrealtime=self.get_xnynzn(now_central,area)
                        
                        self.cross_xn_pub.publish(x_inrealtime)
                        self.cross_yn_pub.publish(y_inrealtime)
                        self.cross_an_pub.publish(z_inrealtime)
                        self.cross_area_pub.publish(area)
                        print "real x,y,z,area is:",x_inrealtime,y_inrealtime,z_inrealtime, area
                        rospy.logerr("real uv info is: %s"%str(now_central))


    def change_uv_to_cartisian(self,point1):
        x=(point1[0]-self.centra_uv[0])*(self.pu/self.f)
        y=(point1[1]-self.centra_uv[1])*(self.pv/self.f)
        return x,y


    def get_xnynzn(self,centroid_point,area_inrealtime):
        z_inrealtime=self.z_dstar*math.sqrt(self.a_dstar/area_inrealtime)
        xg,yg=self.change_uv_to_cartisian(centroid_point)
        x_inrealtime=xg*z_inrealtime
        y_inrealtime=yg*z_inrealtime
        return x_inrealtime,y_inrealtime,z_inrealtime


def main():
    try:
        rospy.init_node("Generation_image_feature")
        rospy.loginfo("Starting generation image features node")
        flag=0
        z_dstar=0.20
        a_dstar=0.0050
        Generation_image_feature=StructurePointxnynanRead(z_dstar,a_dstar)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if Generation_image_feature.rgb_image is not None:
                Generation_image_feature.image_process(Generation_image_feature.rgb_image)
                rate.sleep()
    except KeyboardInterrupt:
        print "Stopping generation image features node"
        cv2.destroyAllWindows()
if __name__=="__main__":
    main()
