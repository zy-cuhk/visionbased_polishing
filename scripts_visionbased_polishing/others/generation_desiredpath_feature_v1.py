#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time,sys
import rospy
# from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Float64
import math
import yaml,os,sys

# o_path="/data/ros/yue_ws_201903/src/visionbased_polishing"
# sys.path.append(o_path) 

from visionbased_polishing.msg import uv

class Sturecuturexdydzdpub():
    def __init__(self):
        self.t=0
        self.udot=-3
        self.vdot=-3

        self.califilename="/home/zy/catkin_ws/src/polishingrobot_lx/visionbased_polishing/yaml/cam_300_industry_20200518.yaml"
        self.file=open(self.califilename)
        self.yamldata=yaml.load(self.file)
        self.f = 0.6245768 #self.yamldata['focal_length'] 
        self.fx = self.yamldata['camera_matrix']['data'][0]
        self.fy = self.yamldata['camera_matrix']['data'][4]
        self.u0 = self.yamldata['camera_matrix']['data'][2]
        self.v0 = self.yamldata['camera_matrix']['data'][5]
        self.pu=self.f/self.fx
        self.pv=self.f/self.fy
        self.camera_center=[self.u0,self.v0]
        
        self.auv=uv()
        self.uv_init=[self.u0,self.v0]

        self.desire_pub=rospy.Publisher("/camera_uv/uvlist", uv, queue_size=10)
        self.xd_pub = rospy.Publisher("/structure_xd", Float64, queue_size=10)
        self.yd_pub = rospy.Publisher("/structure_yd", Float64, queue_size=10)

    def update_u(self,uv_info,udot,flag):
        uv_info1=uv_info[:]
        uv_info1[0]=uv_info[0]+udot*flag
        uv_info1[1]=uv_info[1]
        return uv_info1

    def update_v(self,uv_info,vdot,flag):
        uv_info1=uv_info[:]
        uv_info1[0]=uv_info[0]
        uv_info1[1]=uv_info[1]+vdot*flag
        return uv_info1

    def change_uv_to_cartisian(self,uv):
        x=(uv[0]-self.camera_center[0])/self.fx
        y=(uv[1]-self.camera_center[1])/self.fy
        return x,y

    def generation_desired_paths(self,n):
        uv_info=self.uv_init
        # print("the uv info is:",uv_info)
        for i in range(1,n):            
            if i<50:
                uv_info=self.update_u(uv_info,self.udot,1)
            elif i>=50 and i<100:
                uv_info=self.update_v(uv_info,self.vdot,1)                                
            elif i>=100 and i<150:
                uv_info=self.update_u(uv_info,self.udot,-1)                                
            elif i>=150 and i<=200:
                uv_info=self.update_v(uv_info,self.vdot,-1)
            # print("the uv info is:",uv_info)    
        self.auv.uvinfo = uv_info
        self.desire_pub.publish(self.auv)
        x,y = self.change_uv_to_cartisian(uv_info)
        self.xd_pub.publish(x)
        self.yd_pub.publish(y)
        print "desired x y  is:",x,y
        print "desired uv info is:", uv_info[0], uv_info[1]
        rospy.logerr("desired uv info is: %s"%str(uv_info))

def main():
    rospy.init_node("generation_desired_paths")
    ratet=1
    rate = rospy.Rate(ratet)
    Generation_desired_path=Sturecuturexdydzdpub()
    i=1
    while not rospy.is_shutdown():
        Generation_desired_path.generation_desired_paths(i)
        i=i+1
        if i==200:
            i=1
        rate.sleep()
if __name__=="__main__":
    main()

