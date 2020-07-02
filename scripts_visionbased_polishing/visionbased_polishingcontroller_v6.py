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

o_path="/data/ros/yue_ws_201903/src/visionbased_polishing"
sys.path.append(o_path) 

import scripts_arm.frompitoangle
from scripts_arm.ur5_kinematics import Kinematic
from scripts_arm.hand_in_eye import *
from scripts_arm.trans_methods import *
from scripts_arm.get_arpose_from_ar import *
from scripts_arm.ur5_pose_get import *
from scripts_arm.uv_sub_node import *
from scripts_arm.structure_point_xdydzd_sub import *
from scripts_arm.structure_point_xnynan_sub import *
from scripts_arm.impedance_netf_data_get import *
from scripts_arm.ur_tool_velocity_sub import *

"""
withforce control,just cartesian control with trejactory palnning in retangle,and use structure line with z depth,x depth,y depth
"""
class VisonControl():
    def __init__(self,califilename,urdfname,ratet):
        self.califilename=califilename
        self.urdfname=urdfname

        self.ace=50
        self.vel=0.1
        self.urt=0

        self.detat=0.05 #float(1.0/ratet)
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

        self.extrinc_paramlist=numpy.array([0.0778746546832,0.000796420186253,-0.0853968073946,-0.716503195961,0.00435505650847,-0.694984883822,0.0600017909651])

        self.x_detaff_pub=rospy.Publisher("/x_detaf_info", Float64, queue_size=10)
        self.y_detaff_pub=rospy.Publisher("/y_detaf_info", Float64, queue_size=10)
        self.z_detaff_pub=rospy.Publisher("/z_detaf_info", Float64, queue_size=10)

        self.x_detaffl_pub = rospy.Publisher("/x_detaf_L_info", Float64, queue_size=10)
        self.y_detaffl_pub = rospy.Publisher("/y_detaf_L_info", Float64, queue_size=10)
        self.z_detaffl_pub = rospy.Publisher("/z_detaf_L_info", Float64, queue_size=10)

        self.ur_reader = Urposition()
        self.ur_sub = rospy.Subscriber("/joint_states", JointState, self.ur_reader.callback)
        
        self.tool_get=UrToolVelocityRead()
        self.tool_velocity_sub=rospy.Subscriber("/tool_velocity", TwistStamped, self.tool_get.Ur_tool_velocity_callback)
        self.ur_pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)

        self.netf_reader = NetfData()
        self.netf_sub = rospy.Subscriber("/robotiq_ft_wrench", WrenchStamped, self.netf_reader.callback)

        self.structure_xnynan=StructurePointxnynanRead()
        self.xn_sub = rospy.Subscriber("/cross_line_xsubn", Float64, self.structure_xnynan.structure_point_xn_callback)
        self.yn_sub = rospy.Subscriber("/cross_line_ysubn", Float64, self.structure_xnynan.structure_point_yn_callback)
        self.an_sub = rospy.Subscriber("/cross_line_asubn", Float64, self.structure_xnynan.structure_point_an_callback)

        self.xyd_get=StructurePointXdydzdRead()
        self.xd_sub = rospy.Subscriber("/structure_xd", Float64, self.xyd_get.structure_point_xd_callback)
        self.yd_sub = rospy.Subscriber("/structure_yd", Float64, self.xyd_get.structure_point_yd_callback)

        # self.uv_get=UVRead()
        # self.uv_sub=rospy.Subscriber("/camera_uv/uvlist", uv,self.uv_get.callback)
        
        self.x_error_pub = rospy.Publisher("/feature_x_error", Float64, queue_size=10)
        self.y_error_pub = rospy.Publisher("/feature_y_error", Float64, queue_size=10)
        self.z_error_pub = rospy.Publisher("/feature_z_error", Float64, queue_size=10)
        
        self.x_impedance_error_pub = rospy.Publisher("/x_impedance_error_info", Float64, queue_size=10)
        self.y_impedance_error_pub = rospy.Publisher("/y_impedance_error_info", Float64, queue_size=10)
        self.z_impedance_error_pub = rospy.Publisher("/z_impedance_error_info", Float64, queue_size=10)

    def visionbased_impedancecontroller(self):
        sturucture_point_xnow=self.structure_xnynan.sturucture_point_xn_buf
        sturucture_point_ynow=self.structure_xnynan.sturucture_point_yn_buf
        sturucture_point_anow=self.structure_xnynan.sturucture_point_an_buf
        sturucture_point_xyanow=[sturucture_point_xnow[-1],sturucture_point_ynow[-1],sturucture_point_anow[-1]]
        # print "sturucture_point_xyanow",sturucture_point_xyanow

        structure_point_xdsr=self.xyd_get.sturucture_point_xd_buf
        structure_point_ydsr=self.xyd_get.sturucture_point_yd_buf
        structure_point_adsr=self.structure_xnynan.sturucture_point_an_buf
        structure_point_xyadsr=[structure_point_xdsr[-1],structure_point_ydsr[-1],structure_point_adsr[-1]]
        # print "structure_point_xyadsr",structure_point_xyadsr

        # force_list = self.netf_reader.ave_netf_force_data
        force_list = np.array([0.0,0.0,0.0])
        netf=[force_list[0],force_list[1],force_list[2]]

        q_now = self.ur_reader.ave_ur_pose
        if len(sturucture_point_xnow)!=0 and len(sturucture_point_ynow)!=0 and len(sturucture_point_anow)!=0:
            if len(structure_point_xdsr)!=0 and len(structure_point_ydsr)!=0:
                    joint_speed,vcc=self.get_joint_speed(sturucture_point_xyanow,structure_point_xyadsr, q_now, netf)

                    detaangle = self.get_deta_joint_angle(sturucture_point_xyanow, structure_point_xyadsr, q_now, netf)
                    # print "the deta joints angle are:", detaangle
                    
                    q_pub_next = self.get_joint_angle(q_now,detaangle)
                    # print "the published joints angle are:",q_pub_next

                    ss = "movej([" + str(q_pub_next[0]) + "," + str(q_pub_next[1]) + "," + str(q_pub_next[2]) + "," + str(
                        q_pub_next[3]) + "," + str(q_pub_next[4]) + "," + str(q_pub_next[5]) + "]," + "a=" + str(self.ace) + "," + "v=" + str(
                        self.vel) + "," + "t=" + str(self.urt) + ")"
                    # print("ur5 move joints",ss)
                    self.ur_pub.publish(ss)                    
                    

    def get_joint_speed(self,sturucture_point_xyanow,structure_point_xyadsr,q,f):
        Jacabian_joint,T_06=self.get_jacabian_from_joint(self.urdfname,q,0)
        X=self.get_ur_X()
        jac = tr2jac(X,1)
        jac_b2e=tr2jac(T_06,0)
        inv_X_jac = jac.I
        
        lamdaf=[0.001/2,0.001/2,0.001/2]
        lamdaf_matrix=numpy.matrix([lamdaf[0],0,0,0,lamdaf[1],0,0,0,lamdaf[2]]).reshape((3,3))
        lamdas=[1.0,1.0,-0.01]
        lamdas_matrix=numpy.matrix([lamdas[0],0,0,0,lamdas[1],0,0,0,lamdas[2]]).reshape((3,3))
        fd=[0.0,0.0,0.0]
        detaf = [f[0]-fd[0],f[1]-fd[1],f[2]-fd[2]]
        detas=self.get_feature_error_xyz(sturucture_point_xyanow,structure_point_xyadsr)
        # print("sturucture_point_xyanow",sturucture_point_xyanow)
        # print("structure_point_xyadsr",structure_point_xyadsr)
        # print("detas",detas)

        vc1=lamdas_matrix*numpy.matrix(detas).T
        vc2=lamdaf_matrix*numpy.matrix(detaf).T
        vc=vc1+vc2
        vcc=[vc.tolist()[0][0],vc.tolist()[1][0],vc.tolist()[2][0],0,0,0]
        # print "the camera velocity in camera frame is:",vcc

        ee_speed_in_eeframe = np.dot(inv_X_jac, numpy.matrix(vcc).T)
        v_list = ee_speed_in_eeframe.reshape((1, 6)).tolist()[0]
        flag_list = [1, 1, 1, 0, 0, 0]
        vdot_z = [1.0 * v_list[i] * flag_list[i] for i in range(6)]
        # print "the end effector velocity in end effector frame", vdot_z

        ee_speed_in_base = np.dot(jac_b2e.I, numpy.mat(vdot_z).T)
        j_speed=numpy.dot(Jacabian_joint.I,ee_speed_in_base)
        # print "joints speed are:",j_speed

        # self.x_error_pub.publish(detas[0])
        # self.y_error_pub.publish(detas[1])
        # self.z_error_pub.publish(detas[2])    

        # self.x_detaff_pub.publish(detaf[0])
        # self.y_detaff_pub.publish(detaf[1])
        # self.z_detaff_pub.publish(detaf[2])

        # self.x_detaffl_pub.publish(lamdaf[0]/lamdas[0]*detaf[0])
        # self.y_detaffl_pub.publish(lamdaf[1]/lamdas[1]*detaf[1])
        # self.z_detaffl_pub.publish(lamdaf[2]/lamdas[2]*detaf[2])


        # x_impedance_error = self.tool_get.Ur_tool_velocity_buf[-1][0] - vcc[0]
        # y_impedance_error = self.tool_get.Ur_tool_velocity_buf[-1][1] - vcc[1]
        # z_impedance_error = self.tool_get.Ur_tool_velocity_buf[-1][2] - vcc[2]
        # print("x_impedance_error",x_impedance_error)
        # self.x_impedance_error_pub.publish(x_impedance_error)
        # self.y_impedance_error_pub.publish(y_impedance_error)
        # self.z_impedance_error_pub.publish(z_impedance_error)

        detas1=detas[:]
        # print("detas1:",detas)
        detas1[0]=detas1[0]*self.kx
        detas1[1]=detas1[1]*self.ky
        rospy.logerr("the image feature errors are:%s",str(numpy.matrix(detas1).T))

        return j_speed,vcc
    
    def get_deta_joint_angle(self,sturucture_point_xyanow,structure_point_xyadsr,q,f):
        j_speed,Vcc=self.get_joint_speed(sturucture_point_xyanow,structure_point_xyadsr,q,f)
        # print("joint speed is:",j_speed)
        deta_joint_angle=float(self.detat)*numpy.array(j_speed)
        # print("deta_joint_angle is:",deta_joint_angle)
        return deta_joint_angle


    def get_joint_angle(self,qnow,detajoint):
        q_next=[]
        for i in range(len(detajoint.tolist())):
            q_next.append(detajoint.tolist()[i][0]+qnow[i])
        return q_next

    def get_feature_error_xyz(self,sturucture_point_xyanow,structure_point_xyadsr):
        deta_x=sturucture_point_xyanow[0]-structure_point_xyadsr[0]
        deta_y=sturucture_point_xyanow[1]-structure_point_xyadsr[1]
        deta_z=sturucture_point_xyanow[2]-structure_point_xyadsr[2]

        return [deta_x,deta_y,deta_z]

    def change_uv_to_cartisian(self,uv):
        x=(uv[0]-self.centra_uv[0])/self.kx
        y=(uv[1]-self.centra_uv[1])/self.ky
        return x,y


    def get_jacabian_from_joint(self,urdfname,jointq,flag):
        robot = URDF.from_xml_file(urdfname)
        tree = kdl_tree_from_urdf_model(robot)
        chain = tree.getChain("base_link", "ee_link")
        kdl_kin = KDLKinematics(robot, "base_link", "ee_link")
        J = kdl_kin.jacobian(jointq)
        pose = kdl_kin.forward(jointq)   
        return J,pose


    """obtain feature error"""
    def get_feature_error(self,desireuv,nowuv):
        kk=numpy.mat(nowuv).T-numpy.mat(desireuv).T
        return kk.reshape((1,2))

    def get_ur_X(self):
        aa=get_X_from_ar_quaternion(self.extrinc_paramlist)
        aa=np.mat(aa)
        return aa.reshape((4, 4))


def main():
    rospy.init_node("visionbased_polishingcontroller")
    ratet=5
    rate = rospy.Rate(ratet)                

    urdfname="/data/ros/yue_ws_201903/src/visionbased_polishing/urdf/ur5.urdf"
    filename="/data/ros/yue_ws_201903/src/tcst_pkg/yaml/cam_300_industry_20200518.yaml"
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