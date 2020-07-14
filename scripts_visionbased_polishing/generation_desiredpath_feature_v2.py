#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time,sys
import rospy
from std_msgs.msg import Float64
import math
import yaml,os,sys

from visionbased_polishing.msg import uv

class Sturecuturexdydzdpub():
    def __init__(self):
        self.t=0
        self.udot=-3
        self.vdot=-3
        self.auv=uv()
        self.uv_init=[320.0,240.0]
        self.desire_pub=rospy.Publisher("/camera_uv/uv_desire", uv, queue_size=10)

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

    def generation_desired_paths(self,n):
        uv_info=self.uv_init
        for i in range(1,n):            
            if i<50:
                uv_info=self.update_u(uv_info,self.udot,1)
            elif i>=50 and i<100:
                uv_info=self.update_v(uv_info,self.vdot,1)                                
            elif i>=100 and i<150:
                uv_info=self.update_u(uv_info,self.udot,-1)                                
            elif i>=150 and i<=200:
                uv_info=self.update_v(uv_info,self.vdot,-1)
        self.auv.uvinfo = uv_info
        self.desire_pub.publish(self.auv)
        print "desired uv info is:", uv_info[0], uv_info[1]


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

