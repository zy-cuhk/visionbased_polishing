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
    def __init__(self,califilename,sim,urdfname,netf_zero):
        rospy.init_node("vision_control")
        self.califilename=califilename
        self.sim=sim
        # self.lambda1=lambda1
        self.urdfname=urdfname
        self.netf_zero=netf_zero
        self.x_detaff_pub=rospy.Publisher("/x_detaf_info", Float64, queue_size=10)
        self.y_detaff_pub=rospy.Publisher("/y_detaf_info", Float64, queue_size=10)
        self.z_detaff_pub=rospy.Publisher("/z_detaf_info", Float64, queue_size=10)
        self.x_detaffl_pub = rospy.Publisher("/x_detaf_L_info", Float64, queue_size=10)
        self.y_detaffl_pub = rospy.Publisher("/y_detaf_L_info", Float64, queue_size=10)
        self.z_detaffl_pub = rospy.Publisher("/z_detaf_L_info", Float64, queue_size=10)

        self.f=0.6245768
        self.file=open(self.califilename)
        self.yamldata=yaml.load(self.file)
        self.kx = self.yamldata['camera_matrix']['data'][0]
        self.ky = self.yamldata['camera_matrix']['data'][4]
        self.u0 = self.yamldata['camera_matrix']['data'][2]
        self.v0 = self.yamldata['camera_matrix']['data'][5]
        self.cam = {'kx': self.kx, 'ky': self.ky, "u0": self.u0, "v0": self.v0}
        self.kx = self.cam['kx']
        self.ky = self.cam['ky']
        self.pu=self.f/self.kx
        self.pv=self.f/self.ky
        self.centra_uv=[self.cam['u0'],self.cam['v0']]

        self.intrinc_paramlist=numpy.array([895.957338, 0.000000, 333.621431, 0.000000, 895.348321, 241.448886, 0.000000, 0.000000, 1.000000])
        self.extrinc_paramlist=numpy.array([0.0778746546832,0.000796420186253,-0.0853968073946,-0.716503195961,0.00435505650847,-0.694984883822,0.0600017909651])

        self.ur_reader = Urposition()
        self.ur_sub = rospy.Subscriber("/joint_states", JointState, self.ur_reader.callback)
        self.tool_get=UrToolVelocityRead()
        self.tool_velocity_sub=rospy.Subscriber("/tool_velocity", TwistStamped, self.tool_get.Ur_tool_velocity_callback)
        self.netf_reader = NetfData()
        self.netf_sub = rospy.Subscriber("/robotiq_ft_wrench", WrenchStamped, self.netf_reader.callback)

        self.structure_xnynan=StructurePointxnynanRead()
        self.xn_sub = rospy.Subscriber("/cross_line_xsubn", Float64, self.structure_xnynan.structure_point_xn_callback)
        self.yn_sub = rospy.Subscriber("/cross_line_ysubn", Float64, self.structure_xnynan.structure_point_yn_callback)
        self.an_sub = rospy.Subscriber("/cross_line_asubn", Float64, self.structure_xnynan.structure_point_an_callback)

        self.uv_get=UVRead()
        self.uv_sub=rospy.Subscriber("/camera_uv/uvlist", uv,uv_get.callback)
        xd_get=StructurePointXdydzdRead()
        xd_sub = rospy.Subscriber("/structure_xd", Float64, xd_get.structure_point_xd_callback)
        yd_sub = rospy.Subscriber("/structure_yd", Float64, xd_get.structure_point_yd_callback)

        self.x_error_pub = rospy.Publisher("/feature_x_error", Float64, queue_size=10)
        self.y_error_pub = rospy.Publisher("/feature_y_error", Float64, queue_size=10)
        self.z_error_pub = rospy.Publisher("/feature_z_error", Float64, queue_size=10)
        self.x_impedance_error_pub = rospy.Publisher("/x_impedance_error_info", Float64, queue_size=10)
        self.y_impedance_error_pub = rospy.Publisher("/y_impedance_error_info", Float64, queue_size=10)
        self.z_impedance_error_pub = rospy.Publisher("/z_impedance_error_info", Float64, queue_size=10)

        self.structure_xnynan=StructurePointxnynanRead()
        self.xn_sub = rospy.Subscriber("/cross_line_xsubn", Float64, self.structure_xnynan.structure_point_xn_callback)
        self.yn_sub = rospy.Subscriber("/cross_line_ysubn", Float64, self.structure_xnynan.structure_point_yn_callback)
        self.an_sub = rospy.Subscriber("/cross_line_asubn", Float64, self.structure_xnynan.structure_point_an_callback)    


    def get_jacabian_from_joint(self,urdfname,jointq,flag):
        robot = URDF.from_xml_file(urdfname)
        tree = kdl_tree_from_urdf_model(robot)
        chain = tree.getChain("base_link", "ee_link")
        kdl_kin = KDLKinematics(robot, "base_link", "ee_link")
        q=jointq
        pose = kdl_kin.forward(q) 
        q0=Kinematic()
        if flag==1:
            q_ik=q0.best_sol_for_other_py( [1.] * 6, 0, q0.Forward(q))
        else:
            q_ik = kdl_kin.inverse(pose) 
        if q_ik is not None:
            pose_sol = kdl_kin.forward(q_ik)  
        J = kdl_kin.jacobian(q)
        return J,pose


    """cal image jacbian"""
    def vis2jac(self,uv,z):
        arfx=1/self.pu
        arfy=1/self.pv
        uba=uv[0]-self.cam['u0']
        vba=uv[1]-self.cam['v0']
        L=[[-arfx/z,0,uba/z,1/arfx*uba*vba,-(arfx**2+uba**2)/arfx,vba,0,-arfy/z,vba/z,(arfy**2+vba**2)/arfy,-uba*vba/arfx,-uba]]
        J=numpy.array(L).reshape((2,6))
        return J

    
    """"""
    def vis2jac_mt1(self,uvm,z):
        if len(uvm)>1:
            L=self.vis2jac(uvm[0],z)
            for i in xrange(1,len(uvm)):
                J=numpy.row_stack((L,self.vis2jac(uvm[i],z)))
                L=J
            return J
        else:
            return self.vis2jac(uvm[0],z)

    """obtain feature error"""
    def get_feature_error(self,desireuv,nowuv):
        kk=numpy.mat(nowuv).T-numpy.mat(desireuv).T
        return kk.reshape((1,2))

    """cam speed (udot,vdot)(xdot,ydot,zdot,wxdot,wydot,wzdot)
    get camera frame speed,you must change to ee frame"""
    def get_cam_vdot(self,uvm,z,desireuv,nowuv):
        J=self.vis2jac_mt1(uvm,z)
        JJ=numpy.linalg.pinv(J)
        e=self.get_feature_error(desireuv,nowuv)
        vdot=self.lambda1*numpy.dot(JJ,e.T)
        return vdot

    def get_ur_X(self):
        aa=get_X_from_ar_quaternion(self.extrinc_paramlist)
        aa=np.mat(aa)
        return aa.reshape((4, 4))

    def get_feature_error_xyz(self,armaker_pose,xd,yd,zd):#armaker---x,y,z
        return [armaker_pose[0]-xd,armaker_pose[1]-yd,(armaker_pose[2]-zd)*-1]

    def get_joint_speed(self,armaker_pose,xd,yd,zd,q,xdot,ydot,n,f,fd,lamdaf):#f=[],fd=[],lamdaf=[]
        Jacabian_joint,T_06=self.get_jacabian_from_joint(self.urdfname,q,0)
        
        "the extrinsic parameters are obtained as follows"
        X=self.get_ur_X()
        jac = tr2jac(X,1)
        jac_b2e=tr2jac(T_06,0)
        inv_X_jac = jac.I
        
        lamdaf_matrix=[lamdaf[0],0,0,0,lamdaf[1],0,0,0,lamdaf[2]]
        lamdas1=[-0.5,0,0,0,-0.5,0,0,0,-0.01]

        detaf = [f[0]-fd[0],f[1]-fd[1],f[2]-fd[2]]
        self.x_detaff_pub.publish(f[0] - fd[0])
        self.y_detaff_pub.publish(f[1] - fd[1])
        self.z_detaff_pub.publish(f[2]-fd[2])
        self.x_detaffl_pub.publish(lamdaf[0] / lamdas1[0] * (f[0] - fd[0]))
        self.y_detaffl_pub.publish(lamdaf[1] / lamdas1[4] * (f[1] - fd[1]))
        self.z_detaffl_pub.publish(lamdaf[2]/lamdas1[8]*(f[2]-fd[2]))

        detas=self.get_feature_error_xyz(armaker_pose,xd,yd,zd)
        detas1=detas[:]
        detas1[0]=detas1[0]*self.f/self.pu
        detas1[1]=detas1[1]*self.f/self.pv
        rospy.logerr("the image feature errors are:%s",str(numpy.matrix(detas1).T))
        # print "the image feature errors are:",numpy.matrix(detas).T

        L=[-1,0,0,0,-1,0,0,0,-1]
        Vc=numpy.dot(numpy.matrix(L).reshape((3,3)).I,numpy.matrix(lamdas1).reshape((3,3))*numpy.matrix(detas).T+numpy.dot(numpy.matrix(lamdaf_matrix).reshape((3,3)),numpy.matrix(detaf).T))
        Vcc=[Vc.tolist()[0][0],Vc.tolist()[1][0],Vc.tolist()[2][0],0,0,0]
        print "the camera velocity in camera frame is:",Vcc

        ee_speed_in_eeframe = np.dot(inv_X_jac, numpy.matrix(Vcc).T)
        v_list = ee_speed_in_eeframe.reshape((1, 6)).tolist()[0]

        flag_list = [1, 1, 1, 0, 0, 0]
        vdot_z = [1.0 * v_list[i] * flag_list[i] for i in range(6)]

        ee_speed_in_base = np.dot(jac_b2e.I, numpy.mat(vdot_z).T)
        print "ee speed in base frame is:",ee_speed_in_base

        j_speed=numpy.dot(Jacabian_joint.I,ee_speed_in_base)
        print "Jacabian matrix is:",Jacabian_joint
        print "joints speed are:",j_speed
        return j_speed,Vcc
    
    def get_deta_joint_angular(self,detat,armaker_pose,xd,yd,zd,q,xdot,ydot,n,f,fd,lamda2):
        j_speed,Vcc=self.get_joint_speed(armaker_pose,xd,yd,zd,q,xdot,ydot,n,f,fd,lamda2)
        joint_angular=float(detat)*numpy.array(j_speed)
        return joint_angular

    def get_joint_angular(self,qnow,detajoint):
        listangular=[]
        for i in range(len(detajoint.tolist())):
            listangular.append(detajoint.tolist()[i][0]+qnow[i])
        return listangular

    def change_uv_to_cartisian(self,point1):
        x=(point1[0]-self.centra_uv[0])*(self.pu/self.f)
        y=(point1[1]-self.centra_uv[1])*(self.pv/self.f)
        return x,y

def main():
    urdfname="/data/ros/yue_ws_201903/src/visionbased_polishing/urdf/ur5.urdf"
    filename="/data/ros/yue_ws_201903/src/tcst_pkg/yaml/cam_300_industry_20200518.yaml"

    "the robot arm parameters are set as follows:"
    ace=50
    vel=0.1
    urt=0

    "the adjustable parameters are set as follows:"
    detat=0.05
    z=0.25
    ratet=5
    netf_zero = [0.0,0.0,0.0]
    xdot = 0.01
    ydot = 0.01
    fd=[0.0,0.0,0.0]
    # the original parameter is shown as follows: 
    # lamdaf=[-0.001/2,-0.001/2,-0.001/2]#
    lamdaf=[0.001/2,0.001/2,0.001/2]

    p0=VisonControl(filename,0,urdfname,netf_zero)

    ur_reader = Urposition()
    ur_sub = rospy.Subscriber("/joint_states", JointState, ur_reader.callback)

    structure_xnynan=StructurePointxnynanRead()
    xn_sub = rospy.Subscriber("/cross_line_xsubn", Float64, structure_xnynan.structure_point_xn_callback)
    yn_sub = rospy.Subscriber("/cross_line_ysubn", Float64, structure_xnynan.structure_point_yn_callback)
    an_sub = rospy.Subscriber("/cross_line_asubn", Float64, structure_xnynan.structure_point_an_callback)

    uv_get=UVRead()
    uv_sub=rospy.Subscriber("/camera_uv/uvlist", uv,uv_get.callback)
    xd_get=StructurePointXdydzdRead()
    xd_sub = rospy.Subscriber("/structure_xd", Float64, xd_get.structure_point_xd_callback)
    yd_sub = rospy.Subscriber("/structure_yd", Float64, xd_get.structure_point_yd_callback)

    tool_get=UrToolVelocityRead()
    tool_velocity_sub=rospy.Subscriber("/tool_velocity", TwistStamped, tool_get.Ur_tool_velocity_callback)
    netf_reader = NetfData()
    netf_sub = rospy.Subscriber("/robotiq_ft_wrench", WrenchStamped, netf_reader.callback)

    x_error_pub = rospy.Publisher("/feature_x_error", Float64, queue_size=10)
    y_error_pub = rospy.Publisher("/feature_y_error", Float64, queue_size=10)
    z_error_pub = rospy.Publisher("/feature_z_error", Float64, queue_size=10)

    z_impedance_error_pub = rospy.Publisher("/z_impedance_error_info", Float64, queue_size=10)
    y_impedance_error_pub = rospy.Publisher("/y_impedance_error_info", Float64, queue_size=10)
    x_impedance_error_pub = rospy.Publisher("/x_impedance_error_info", Float64, queue_size=10)

    ur_pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)

    z_depth_pub = rospy.Publisher("/camera_depth", Float64, queue_size=10)
    now_uv_pub = rospy.Publisher("/nowuv_info", uv, queue_size=10)

    rate = rospy.Rate(ratet)                
    count=1
    marker_zero=[]
    # while count<30:
    while not rospy.is_shutdown():
        try:
            sturucture_point_xn=structure_xnynan.sturucture_point_xn_buf
            sturucture_point_yn=structure_xnynan.sturucture_point_yn_buf
            sturucture_point_an=structure_xnynan.sturucture_point_an_buf

            "the modification part 1"
            # force_list = netf_reader.ave_netf_force_data
            force_list = np.array([0.0,0.0,0.0])
            
            if len(sturucture_point_xn)!=0 and len(sturucture_point_yn)!=0 and len(sturucture_point_an)!=0:
                "the modification part 2"
                # if len(xd_get.sturucture_point_xd_buf)!=0 and len(xd_get.sturucture_point_yd_buf)!=0:
                    
                "the modification part 3"
                # xd=xd_get.sturucture_point_xd_buf[-1]
                # yd=xd_get.sturucture_point_yd_buf[-1]
                # zd=0.15

                xd=305
                yd=255
                point1=np.array([xd,yd])
                xd,yd=p0.change_uv_to_cartisian(point1)

                zd = sturucture_point_an[-1]
                xnynan=[sturucture_point_xn[-1],sturucture_point_yn[-1],sturucture_point_an[-1]]
                print "the image features are:",xnynan
                print "the desired feature is:", xd,yd,zd
                print "the force sensor values are: ",force_list

                if len(force_list) != 0 and len(tool_get.Ur_tool_velocity_buf)!=0:
                    "obtain real-time force, desired force and joint speed"
                    netf=[force_list[0],force_list[1],force_list[2]]

                    q_now = ur_reader.ave_ur_pose
                    joint_speed,vcc=p0.get_joint_speed(xnynan, xd, yd, zd, q_now, xdot, 0 * ydot, count, netf, fd, lamdaf)
                    print "joint speed is:", joint_speed
                    # print "joint_speed,vcc\n", joint_speed,vcc
                    if abs(joint_speed[0])<=100 and abs(joint_speed[1])<=100 and abs(joint_speed[2])<=100 and abs(joint_speed[3])<=100 and abs(joint_speed[4])<=100 and abs(joint_speed[5])<=100:
                        """publish feature error"""
                        detas = p0.get_feature_error_xyz(xnynan, xd, yd, zd)
                        x_error_pub.publish(detas[0])
                        y_error_pub.publish(detas[1])
                        z_error_pub.publish(-1*detas[2])

                        """publish impedance error"""
                        x_impedance_error = tool_get.Ur_tool_velocity_buf[-1][0] - vcc[0]
                        y_impedance_error = tool_get.Ur_tool_velocity_buf[-1][1] - vcc[1]
                        z_impedance_error=tool_get.Ur_tool_velocity_buf[-1][2]-vcc[2]
                        x_impedance_error_pub.publish(x_impedance_error)
                        y_impedance_error_pub.publish(y_impedance_error)
                        z_impedance_error_pub.publish(z_impedance_error)


                        """publish movej joints value"""
                        print "the present joints angle are:",q_now
                        detaangular = p0.get_deta_joint_angular(detat, xnynan, xd, yd, zd, q_now, xdot, 0*ydot, count,netf,fd,lamdaf)
                        q_pub_now=p0.get_joint_angular(q_now,detaangular)
                        print "the deta joints angular are:",detaangular
                        print "the published joints angle are:",q_pub_now
                        ss = "movej([" + str(q_pub_now[0]) + "," + str(q_pub_now[1]) + "," + str(q_pub_now[2]) + "," + str(
                            q_pub_now[3]) + "," + str(q_pub_now[4]) + "," + str(q_pub_now[5]) + "]," + "a=" + str(ace) + "," + "v=" + str(
                            vel) + "," + "t=" + str(urt) + ")"
                        ur_pub.publish(ss)

                        count += 1
                        print "count--------",count
                    rate.sleep()
                # else:
                #     print "the desired value is not obtained"
        except:
            continue

if __name__=="__main__":
    main()