#!/usr/bin/env python

"""
ref:
ros robot development practice
Computer vision
chapter 7 pic 7-14
"""
import numpy as np
import math
import cv2
import time
import rospy
from ur5_planning.msg import uv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers

import cv2

uv_center_pos = [305,255]
radius = 150
blue = (255, 0, 0)
red = (0, 0, 255)


class cam_canvas():
    """  default open  came:  video1"""
    def __init__(self, name=""):
        self.name = "opencv_reader"
        self.design_traj_list = []
        self.real_traj_list = []
        self.design_uv = []
        self.now_uv = []
        self.bridge = CvBridge()
        self.img = []

    def Init_node(self):
        rospy.init_node(self.name)
        design_uv_sub = rospy.Subscriber("/camera_uv/uvlist", uv, self.callback_design_uv)
        img_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback_usbcam)
        img_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        return design_uv_sub,img_sub,img_pub

    def callback_usbcam(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

    def callback_design_uv(self, msg):
        self.design_uv = msg.uvinfo

    def add_in_real_traj_list(self, uv_info):
        self.real_traj_list.append((int(uv_info[0]), int(uv_info[1])))

    def add_in_design_traj_list(self, uv_info):
        self.design_traj_list.append((int(uv_info[0]), int(uv_info[1])))

    def draw_traj(self, frame, tnum, pos_list, clor, thick_val):
        for i in range(0, tnum):
            frame = cv2.line(frame, pos_list[i], pos_list[i + 1], color=clor, thickness=thick_val)
        return frame

    def frame_operate(self, frame, tnum, design_uv, now_uv):
        self.add_in_real_traj_list( now_uv )
        self.add_in_design_traj_list( design_uv )
        frame = self.draw_traj(frame, tnum, self.design_traj_list, red, 5 )
        frame = self.draw_traj(frame, tnum, self.real_traj_list, blue, 2 )
        return frame



    def cam_live_stream(self, img_pub, name="on-live-image"):
        frame_num = 0
        rate=rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                design_uv = self.design_uv
                now_uv = [0,0]
                print("new_data:", design_uv)
                frame = np.array( self.img )
                print("opencv, new data from rosnode :", now_uv)
                frame = self.frame_operate(frame, frame_num, design_uv, now_uv)
                frame_num += 1
                img_pub.publish(self.bridge.cv2_to_imgmsg( frame, "bgr8"))
            except:
                continue
            rate.sleep()


    def frame_operate1(self, frame, tnum, design_uv):
        self.real_traj_list.append((int(design_uv[0]), int(design_uv[1])))
        for i in range(0, tnum):
            frame = cv2.line(frame, self.real_traj_list[i], self.real_traj_list[i+1], color=red, thickness=5)
        return frame


    def cam_live_stream1(self, img_pub, name="on-live-image"):
        frame_num = 0
        rate=rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                design_uv = self.design_uv
                print("new_data:", design_uv)
                frame = np.array( self.img )
                frame = self.frame_operate(frame, frame_num, design_uv)
                frame_num += 1
                img_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            except:
                continue
            rate.sleep()
    
    def cam_live_stream2(self,img_pub):
        frame_num = 0
        rate=rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                design_uv = self.design_uv
                print("new received uv data:", design_uv)
                frame = np.array( self.img )
                self.real_traj_list.append((int(design_uv[0]), int(design_uv[1])))
                for i in range(0, frame_num):
                    frame = cv2.line(frame, self.real_traj_list[i], self.real_traj_list[i+1], color=red, thickness=5)
                frame_num += 1
                img_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            except:
                continue
            rate.sleep()
    

def main():
    cam_canv = cam_canvas()
    design_uv_sub, img_sub, img_pub = cam_canv.Init_node()
    cam_canv.cam_live_stream(img_pub)

if __name__ == "__main__":
    main()