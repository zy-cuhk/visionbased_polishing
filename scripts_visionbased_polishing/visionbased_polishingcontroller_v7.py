#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy
from numpy import matlib,linalg
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
import rospy
import yaml,os,sys

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped,TwistStamped
from ur5_planning.msg import uv
from math import *

o_path="/home/zy/catkin_ws/src/polishingrobot_lx/visionbased_polishing"
sys.path.append(o_path) 

import scripts_arm.frompitoangle
from scripts_arm.ur5_kinematics import Kinematic
from scripts_arm.hand_in_eye import *
from scripts_arm.trans_methods import *
from scripts_arm.ur5_pose_get import *
from scripts_arm.uv_sub_node import *
from scripts_arm.structure_point_xdydzd_sub import *
from scripts_arm.structure_point_xnynan_sub import *
from scripts_arm.impedance_netf_data_get import *
from scripts_arm.ur_tool_velocity_sub import *

class VisonControl():
    def __init__(self,califilename,urdfname,ratet):
        self.califilename=califilename
        self.urdfname=urdfname

        self.ace=50
        self.vel=0.1
        self.urt=0

        self.detat=float(1.0/ratet)
        self.file=open(self.califilename)
        self.yamldata=yaml.load(self.file)
        self.f = 0.6245768 #self.yamldata['focal_length']
        self.kx = self.yamldata['camera_matrix']['data'][0]
        self.ky = self.yamldata['camera_matrix']['data'][4]
        self.u0 = self.yamldata['camera_matrix']['data'][2]
        self.v0 = self.yamldata['camera_matrix']['data'][5]
        self.pu=self.f/self.kx
        self.pv=self.f/self.ky
        self.centra_uv=[self.u0,self.v0]

        self.ur_reader = Urposition()
        self.ur_sub = rospy.Subscriber("/joint_states", JointState, self.ur_reader.callback)
        self.tool_get=UrToolVelocityRead()
        self.tool_velocity_sub=rospy.Subscriber("/tool_velocity", TwistStamped, self.tool_get.Ur_tool_velocity_callback)
        self.ur_pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)
        self.netf_reader = NetfData()
        self.netf_sub = rospy.Subscriber("/robotiq_ft_wrench", WrenchStamped, self.netf_reader.callback)

        # self.uv_get=UVRead()
        # self.uv_sub=rospy.Subscriber("/camera_uv/uvlist", uv,self.uv_get.callback)
        
        self.x_error_pub = rospy.Publisher("/feature_x_error", Float64, queue_size=10)
        self.y_error_pub = rospy.Publisher("/feature_y_error", Float64, queue_size=10)
        self.z_error_pub = rospy.Publisher("/feature_z_error", Float64, queue_size=10)
        
        self.x_impedance_error_pub = rospy.Publisher("/x_impedance_error_info", Float64, queue_size=10)
        self.y_impedance_error_pub = rospy.Publisher("/y_impedance_error_info", Float64, queue_size=10)
        self.z_impedance_error_pub = rospy.Publisher("/z_impedance_error_info", Float64, queue_size=10)

    def visionbased_impedancecontroller(self):
        "step 1: receiving the data"
        sturucture_point_xyanow=[sturucture_point_xnow[-1],sturucture_point_ynow[-1],sturucture_point_anow[-1]]
        # print "sturucture_point_xyanow",sturucture_point_xyanow
        structure_point_xyadsr=[structure_point_xdsr[-1],structure_point_ydsr[-1],structure_point_adsr[-1]]
        # print "structure_point_xyadsr",structure_point_xyadsr
        force_list = self.netf_reader.ave_netf_force_data
        f=[force_list[0],force_list[1],force_list[2]]
        print("netf is:",f)
        q_now = self.ur_reader.now_ur_pos
        print("q_now is:",q_now)
        
        "step 2: vision based controller"
        lamdas=[-1.0,-1.0,0]
        lamdas_matrix=numpy.matrix([lamdas[0],0,0,0,lamdas[1],0,0,0,lamdas[2]]).reshape((3,3))
        deta_x=sturucture_point_xyanow[0]-structure_point_xyadsr[0]
        deta_y=sturucture_point_xyanow[1]-structure_point_xyadsr[1]
        deta_z=sturucture_point_xyanow[2]-structure_point_xyadsr[2]
        detas= [deta_x,deta_y,deta_z]
        # vc1=lamdas_matrix*numpy.matrix(detas).T
        vc1=0.0        

        lamdaf=[0.0,0.0,0.0001]
        lamdaf_matrix=numpy.matrix([lamdaf[0],0,0,0,lamdaf[1],0,0,0,lamdaf[2]]).reshape((3,3))
        fd=[0.0,0.0,-10.0]
        print("fd is",fd)
        detaf = [f[0]-fd[0],f[1]-fd[1],f[2]-fd[2]]
        print("detaf",detaf)
        vc2=lamdaf_matrix*numpy.matrix(detaf).T

        vc=vc1+vc2
        vcc=[vc.tolist()[0][0],vc.tolist()[1][0],vc.tolist()[2][0],0,0,0]
        # print "the camera velocity in camera frame is:",vcc

        X=numpy.array([[0.0,1.0,0.0,0.0],[-1.0,0.0,0.0,+0.12],[0.0,0.0,1.0,+0.09],[0.0,0.0,0.0,1.0]])
        # print("X is",X)
        jac = tr2jac(X,1)
        # print("jac is:",jac)
        inv_X_jac = jac.I
        # print("inv_X_jac is",inv_X_jac)
        ee_speed_in_eeframe = np.dot(inv_X_jac, numpy.matrix(vcc).T)
        v_list = ee_speed_in_eeframe.reshape((1, 6)).tolist()[0]
        flag_list = [1, 1, 1, 0, 0, 0]
        vdot_z = [1.0 * v_list[i] * flag_list[i] for i in range(6)]
        # print "the end effector velocity in end effector frame", vdot_z

        robot = URDF.from_xml_file(urdfname)
        kdl_kin = KDLKinematics(robot, "base_link", "tool0")
        Jacabian_joint = kdl_kin.jacobian(jointq)
        T_06 = kdl_kin.forward(jointq)   
        #print("Jacabian_joint is",Jacabian_joint)
        jac_b2e=tr2jac(T_06,0)
        # print("jac_b2e is",jac_b2e)
        ee_speed_in_base = np.dot(jac_b2e.I, numpy.mat(vdot_z).T)
        j_speed=numpy.dot(Jacabian_joint.I,ee_speed_in_base)
        # print "joints speed are:",j_speed

        detas1=detas[:]
        # print("detas1:",detas)
        detas1[0]=detas1[0]*self.kx
        detas1[1]=detas1[1]*self.ky
        rospy.logerr("the image feature errors are:%s",str(numpy.matrix(detas1).T))

        "step 3: output the data"
        deta_joint_angle=float(self.detat)*numpy.array(joint_speed)
        print "the deta joints angle are:", deta_joint_angle
        q_pub_next=[]
        for i in range(len(deta_joint_angle.tolist())):
            q_pub_next.append(deta_joint_angle.tolist()[i][0]+q_now[i])
        print "the published joints angle are:",q_pub_next
        ss = "movej([" + str(q_pub_next[0]) + "," + str(q_pub_next[1]) + "," + str(q_pub_next[2]) + "," + str(
            q_pub_next[3]) + "," + str(q_pub_next[4]) + "," + str(q_pub_next[5]) + "]," + "a=" + str(self.ace) + "," + "v=" + str(
            self.vel) + "," + "t=" + str(self.urt) + ")"
        # print("ur5 move joints",ss)
        self.ur_pub.publish(ss)                    
    
    def change_uv_to_cartisian(self,uv):
        x=(uv[0]-self.centra_uv[0])/self.kx
        y=(uv[1]-self.centra_uv[1])/self.ky
        return x,y

    def get_feature_error(self,desireuv,nowuv):
        kk=numpy.mat(nowuv).T-numpy.mat(desireuv).T
        return kk.reshape((1,2))



def main():
    rospy.init_node("visionbased_polishingcontroller")
    ratet=5
    rate = rospy.Rate(ratet)                

    urdfname="/home/zy/catkin_ws/src/polishingrobot_lx/visionbased_polishing/urdf/ur5.urdf"
    filename="/home/zy/catkin_ws/src/polishingrobot_lx/visionbased_polishing/yaml/cam_300_industry_20200518.yaml"
    visionbased_polishing=VisonControl(filename,urdfname,ratet)

    while not rospy.is_shutdown():
    # for i in range(1,100):
        try:
            visionbased_polishing.visionbased_impedancecontroller()
            rate.sleep()
        except:
            continue

if __name__=="__main__":
    main()







