#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time,sys
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Float64
import math

o_path="/data/ros/yue_ws_201903/src/visionbased_polishing"
sys.path.append(o_path) 

from scripts_arm.get_arpose_from_ar import *
from scripts_arm.structure_point_xnynan_sub import *
from visionbased_polishing.msg import uv

class Sturecuturexdydzdpub():
    def __init__(self):
        self.t=0
        self.udot=1.5
        self.vdot=1.5

        self.f=0.6245768      
        self.califilename="/data/ros/yue_ws_201903/src/tcst_pkg/yaml/cam_300_industry_20200518.yaml"
        self.file=open(self.califilename)
        self.yamldata=yaml.load(self.file)
        self.fx = self.yamldata['camera_matrix']['data'][0]
        self.fy = self.yamldata['camera_matrix']['data'][4]
        self.u0 = self.yamldata['camera_matrix']['data'][2]
        self.v0 = self.yamldata['camera_matrix']['data'][5]
        self.pu=self.f/self.fx
        self.pv=self.f/self.udotudotudot
        self.camera_center=[self.u0,self.v0]
        self.auv=uv()
        self.uv_init=[self.u0,self.v0]

        self.desire_pub=rospy.Publisher("/camera_uv/uvlist", uv, queue_size=10)
        self.structure_xnynan=StructurePointxnynanRead()
        self.xn_sub = rospy.Subscriber("/cross_line_xsubn", Float64, self.structure_xnynan.structure_point_xn_callback)
        self.yn_sub = rospy.Subscriber("/cross_line_ysubn", Float64, self.structure_xnynan.structure_point_yn_callback)
        self.an_sub = rospy.Subscriber("/cross_line_asubn", Float64, self.structure_xnynan.structure_point_an_callback)
        self.xd_pub = rospy.Publisher("/structure_xd", Float64, queue_size=10)
        self.yd_pub = rospy.Publisher("/structure_yd", Float64, queue_size=10)

    def update_u(self,uv_info,udot,flag):
        uv_info[0]=uv_info[0]+udot*flag
        uv_info[1]=uv_info[1]
        return uv_info

    def update_v(self,uv_info,vdot,flag):
        uv_info[0]=uv_info[0]
        uv_info[1]=uv_info[1]+vdot*flag
        return uv_info

    def change_uv_to_cartisian(self,uv):
        x=(uv[0]-self.centra_uv[0])/self.fx
        y=(uv[1]-self.centra_uv[1])/self.fy
        return x,y

    def generation_desired_paths(self,n):
        uv_info=self.uv_init
        for i=1:1:n:            
            if i<50:
                uv_info=update_u(uv_info,udot,1)
            elif i>=50 and i<100:
                uv_info=update_v(uv_info,vdot,1)                                
            elif i>=100 and i<150:
                uv_info=update_u(uv_info,udot,-1)                                
            elif i>=150 and i<=200:
                uv_info=update_v(uv_info,vdot,-1)                                
        self.auv.uvinfo = uv_info
        self.desire_pub.publish(auv)
        x,y = self.change_uv_to_cartisian(self,uv)
        self.xd_pub.publish(x)
        self.yd_pub.publish(y)
        print("x is: ", x)
        print "y is: ", y
        print "n-----", n

def main():
    rospy.init_node("generation_desired_paths")
    ratet=1
    rate = rospy.Rate(ratet)
    Generation_desired_path.Sturecuturexdydzdpub()
    while not rospy.is_shutdown():
        try:
            for i in range(1,200):
                Generation_desired_path.generation_desired_paths(i)
        except KeyboardInterrupt:
            sys.exit()

        rate.sleep()
if __name__=="__main__":
    main()

