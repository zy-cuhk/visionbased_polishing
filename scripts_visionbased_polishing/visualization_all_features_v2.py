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
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.design_uv_sub = rospy.Subscriber("/camera_uv/uvlist", uv, self.callback_design_uv)
        self.rgb_image=None
        self.real_traj_list = []
        self.radius=5

    def callback(self,data):
        try:
            video_capture=self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.rgb_image = video_capture.copy()
        except CvBridgeError as e:
            print(e)

    def callback_design_uv(self, msg):
        self.design_uv = msg.uvinfo

    def process_rgb_image(self,image,image_num):
        if image is not None:
            "image processing"
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            kernel1 = cv2.getStructuringElement(cv2.MORPH_RECT,(5, 5))
            eroded1 = cv2.erode(gray,kernel1)
            dilated1 = cv2.dilate(eroded1,kernel1)
            dilated1 = cv2.dilate(dilated1,kernel1)
            eroded1 = cv2.erode(dilated1,kernel1)
            blurred = cv2.GaussianBlur(gray, (11,11), 0)
            thresh = cv2.threshold(blurred, 205, 255, cv2.THRESH_BINARY)[1]
            cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
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
            # print("the central points is located at:",now_central)

            "draw and publish lines and circles on image"
            # print("extLeft is:",extLeft)
            cv2.line(image, extLeft, extTop, [0, 255, 0], 2)
            cv2.line(image, extLeft, extBot, [0, 255, 0], 2)
            cv2.line(image, extRight, extTop, [0, 255, 0], 2)
            cv2.line(image, extRight, extBot, [0, 255, 0], 2)
            cv2.drawContours(image, [c], -1, (0, 255, 255), 2)
            cv2.circle(image, extLeft, self.radius, (0, 0, 255), -1)
            cv2.circle(image, now_central, self.radius, (0, 0, 255), -1)
            design_uv = self.design_uv
            # print("new received uv data:", design_uv)
            # self.real_traj_list.append((int(design_uv[0]), int(design_uv[1])))
            self.real_traj_list.append((int(design_uv[0]),int(design_uv[1])))
            # print("real_traj_list is:",self.real_traj_list)
            # print("the length of real_traj_list is:",len(self.real_traj_list))
            # print("the image number is:",image_num)
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
            Image_features_visualization.process_rgb_image(Image_features_visualization.rgb_image,image_num)
            image_num += 1            
            rate.sleep()
    except KeyboardInterrupt:
        print "Shutting down visualizing image features"
        cv2.destroyAllWindows()
if __name__=="__main__":
    main()
