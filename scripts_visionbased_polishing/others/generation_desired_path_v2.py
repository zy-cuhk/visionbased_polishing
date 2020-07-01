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
    def __init__(self,nodename,xdtopicname,ydtopicname,radius):
        self.nodename=nodename
        self.xdtopicname=xdtopicname
        self.ydtopicname = ydtopicname
        self.radius=radius
        self.cont=10
        rospy.init_node(self.nodename)
        self.desire_pub=rospy.Publisher("/camera_uv/uvlist", uv, queue_size=10)
    def Init_node(self):
        xd_pub = rospy.Publisher(self.xdtopicname, Float64, queue_size=10)
        yd_pub = rospy.Publisher(self.ydtopicname, Float64, queue_size=10)
        return xd_pub,yd_pub

    def get_xd_line(self,marker_info,xdot,t):
        xd = marker_info[0] + xdot * t
        return  xd

    def get_yd_line(self,marker_info,ydot,t):
        yd = marker_info[1] + ydot * t
        return  yd

def main():
    uv0=Sturecuturexdydzdpub("structure_design_pub","/structure_xd","/structure_yd",150)

    structure_xnynan=StructurePointxnynanRead()
    xn_sub = rospy.Subscriber("/cross_line_xsubn", Float64, structure_xnynan.structure_point_xn_callback)
    yn_sub = rospy.Subscriber("/cross_line_ysubn", Float64, structure_xnynan.structure_point_yn_callback)
    an_sub = rospy.Subscriber("/cross_line_asubn", Float64, structure_xnynan.structure_point_an_callback)
    xd_pub,yd_pub=uv0.Init_node()
    ratet=1
    rate = rospy.Rate(ratet)

    f=0.6245768      
    califilename="/data/ros/yue_ws_201903/src/tcst_pkg/yaml/cam_300_industry_20200518.yaml"
    file=open(califilename)
    yamldata=yaml.load(file)
    fx = yamldata['camera_matrix']['data'][0]
    fy = yamldata['camera_matrix']['data'][4]
    u0 = yamldata['camera_matrix']['data'][2]
    v0 = yamldata['camera_matrix']['data'][5]
    pu=f/fx
    pv=f/fy
    camera_center=[u0,v0]

    t=0
    xdot=0.0020
    ydot=0.0023
    marker_zero=[-0.0898,-0.097]
    marker_zero_1=[]
    marker_zero_2 = []  
    marker_zero_3 = []
    xdd=0
    ydd=0
    flag=0
    auv=uv()
    while not rospy.is_shutdown():
        try:
            if t<50:
                xd=uv0.get_xd_line(marker_zero,xdot,t)
                xd_pub.publish(xd)
                yd_pub.publish(marker_zero[1])

                uv1=fx*xd+camera_center[0]
                uv2=fy*marker_zero[1]+camera_center[1]
                auv.uvinfo =[uv1,uv2]
                uv0.desire_pub.publish(auv)
                t += 1
                if t==50:
                    marker_zero_1=[xd,marker_zero[1],0]
                print("xd-----", xd)
                print "yd-----", yd
                print "t-----", t
            elif t>=50 and t<100:
                xd=marker_zero_1[0]
                xd_pub.publish(xd)
                yd = uv0.get_yd_line(marker_zero_1, ydot, (t-50)%50)
                yd_pub.publish(yd)
                uv1=fx*xd+camera_center[0]
                uv2=fy*yd+camera_center[1]
                auv.uvinfo =[uv1,uv2]
                uv0.desire_pub.publish(auv)
                t += 1
                if t==100:
                    ydd=yd
                    marker_zero_2=[marker_zero_1[0],yd,0]
                print "ydd-----", yd
                print "xdd-----",xd
                print "t-----",t
            elif t>=100 and t<150:
                xd = uv0.get_xd_line(marker_zero_2, -xdot, (t-100)%50)
                xd_pub.publish(xd)
                yd_pub.publish(ydd)
                yd=ydd
                uv1=fx*xd+camera_center[0]
                uv2=fy*yd+camera_center[1]
                auv.uvinfo =[uv1,uv2]
                uv0.desire_pub.publish(auv)
                t += 1
                if t==150:
                    xdd=xd
                    marker_zero_3=[xd,marker_zero_2[1],0]
                print "ydd-----", yd
                print "xdd-----",xd
                print "t-----",t
            elif t>=150 and t<200:
                xd_pub.publish(xdd)
                xd=xdd
                yd = uv0.get_yd_line(marker_zero_3, -ydot, (t-150)%50)
                yd_pub.publish(yd)
                uv1=xd/(1/fx)+camera_center[0]
                uv2=yd/(1/fy)+camera_center[1]
                auv.uvinfo =[uv1,uv2]
                uv0.desire_pub.publish(auv)
                t += 1
                print "ydd-----", yd
                print "xdd-----",xd
                print "t-----",t
            else:
                t=0
                print "cnt is over-----"
        except KeyboardInterrupt:
            sys.exit()

        rate.sleep()
if __name__=="__main__":
    main()

