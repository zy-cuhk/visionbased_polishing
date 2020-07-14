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
from visionbased_polishing.msg import uv

red = (0, 0, 255)

class All_features_visualization():
    def __init__(self):
        self.radius=5
        self.bridge = CvBridge()
        self.rgb_image=None
        self.real_traj_list = []

        self.design_uv = []
        self.now_uv = []

        self.uv_leftpoint = []
        self.uv_rightpoint = []
        self.uv_toppoint = []
        self.uv_botpoint = []

        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)

        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback_img)
        self.uv_desire_sub = rospy.Subscriber("/camera_uv/uv_desire", uv, self.callback_desire_uv)
        self.uv_now_sub=rospy.Subscriber("/camera_uv/uv_now", uv, self.callback_now_uv)

        self.uv_leftpoint_sub=rospy.Subscriber("/camera_uv/uv_leftpoint", uv, self.callback_leftpoint_uv)
        self.uv_rightpoint_sub=rospy.Subscriber("/camera_uv/uv_rightpoint", uv, self.callback_rightpoint_uv)
        self.uv_toppoint_sub=rospy.Subscriber("/camera_uv/uv_toppoint", uv, self.callback_toppoint_uv)
        self.uv_botpoint_sub=rospy.Subscriber("/camera_uv/uv_botpoint", uv, self.callback_botpoint_uv)

    def callback_img(self,data):
        try:
            video_capture=self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.rgb_image = video_capture.copy()
        except CvBridgeError as e:
            print(e)

    def callback_desire_uv(self, msg):
        self.design_uv=[]
        self.design_uv.append(int(msg.uvinfo[0]))
        self.design_uv.append(int(msg.uvinfo[1]))

    def callback_now_uv(self, msg):
        self.now_uv=[]
        self.now_uv.append(int(msg.uvinfo[0]))
        self.now_uv.append(int(msg.uvinfo[1]))

    def callback_leftpoint_uv(self,msg):
        self.uv_leftpoint=[]
        self.uv_leftpoint.append(int(msg.uvinfo[0]))
        self.uv_leftpoint.append(int(msg.uvinfo[1]))
    
    def callback_rightpoint_uv(self,msg):
        self.uv_rightpoint=[]
        self.uv_rightpoint.append(int(msg.uvinfo[0]))
        self.uv_rightpoint.append(int(msg.uvinfo[1]))

    def callback_toppoint_uv(self,msg):
        self.uv_toppoint=[]
        self.uv_toppoint.append(int(msg.uvinfo[0]))
        self.uv_toppoint.append(int(msg.uvinfo[1]))

    def callback_botpoint_uv(self,msg):
        self.uv_botpoint=[]
        self.uv_botpoint.append(int(msg.uvinfo[0]))
        self.uv_botpoint.append(int(msg.uvinfo[1]))

    def process_rgb_image(self,image,image_num):
        extLeft = tuple(self.uv_leftpoint)
        extRight = tuple(self.uv_rightpoint)
        extTop = tuple(self.uv_toppoint)
        extBot = tuple(self.uv_botpoint)
        now_central = tuple(self.now_uv)
        design_uv = self.design_uv

        if len(extLeft)!=0 and len(extRight)!=0 and len(extTop)!=0 and len(extBot)!=0 and len(now_central)!=0 and len(design_uv)!=0:
            cv2.line(image, extLeft, extTop, [0, 255, 0], 2)
            cv2.line(image, extLeft, extBot, [0, 255, 0], 2)
            cv2.line(image, extRight, extTop, [0, 255, 0], 2)
            cv2.line(image, extRight, extBot, [0, 255, 0], 2)
            cv2.circle(image, extLeft, self.radius, (0, 0, 255), -1)
            cv2.circle(image, now_central, self.radius, (0, 0, 255), -1)

            self.real_traj_list.append((int(design_uv[0]),int(design_uv[1])))
            if len(self.real_traj_list)>1:
                for i in range(0, image_num-1):
                    cv2.line(image, self.real_traj_list[i], self.real_traj_list[i+1], color=(0, 0, 255), thickness=5)
            cv2.circle(image, self.real_traj_list[image_num-1], self.radius, (255, 0, 0), -1)
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
            except CvBridgeError as e:
                print e
            
def main():
    try:
        rospy.init_node("Visualization_image_features")
        rospy.loginfo("Starting visualizing image features")
        Image_features_visualization=All_features_visualization()
        rate = rospy.Rate(1)
        image_num=0
        while not rospy.is_shutdown():
            if Image_features_visualization.rgb_image is not None:
                Image_features_visualization.process_rgb_image(Image_features_visualization.rgb_image,image_num)
                image_num += 1            
                rate.sleep()
    except KeyboardInterrupt:
        print "Shutting down visualizing image features"
        cv2.destroyAllWindows()
if __name__=="__main__":
    main()
