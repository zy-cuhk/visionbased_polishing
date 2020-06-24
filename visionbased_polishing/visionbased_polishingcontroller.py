#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy
from numpy import matlib,linalg
#Import the module
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
#import cv2
import rospy
import yaml,os,sys
# from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped,TwistStamped
from ur5_planning.msg import uv

o_path="/data/ros/yue_ws_201903/src/visionbased_polishing"
# print(o_path)
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
        self.califilename=califilename
        self.sim=sim
        # self.lambda1=lambda1
        self.urdfname=urdfname
        self.netf_zero=netf_zero
        rospy.init_node("vision_control")
    #if flag=1,use our kinematics for inverse
    def get_jacabian_from_joint(self,urdfname,jointq,flag):
        #robot = URDF.from_xml_file("/data/ros/ur_ws/src/universal_robot/ur_description/urdf/ur5.urdf")
        robot = URDF.from_xml_file(urdfname)
        tree = kdl_tree_from_urdf_model(robot)
        # print tree.getNrOfSegments()
        chain = tree.getChain("base_link", "ee_link")
        # print chain.getNrOfJoints()
        # forwawrd kinematics
        kdl_kin = KDLKinematics(robot, "base_link", "ee_link")
        q=jointq
        #q = [0, 0, 1, 0, 1, 0]
        pose = kdl_kin.forward(q)  # forward kinematics (returns homogeneous 4x4 matrix)
        # print pose
        #print list(pose)
        q0=Kinematic()
        if flag==1:
            q_ik=q0.best_sol_for_other_py( [1.] * 6, 0, q0.Forward(q))
        else:
            q_ik = kdl_kin.inverse(pose)  # inverse kinematics
        # print "----------iverse-------------------\n", q_ik

        if q_ik is not None:
            pose_sol = kdl_kin.forward(q_ik)  # should equal pose
            print "------------------forward ------------------\n",pose_sol

        J = kdl_kin.jacobian(q)
        #print 'J:', J
        return J,pose
    #kx=f/px,ky=f/py
    #sim=1,use camera default from Macine Vision Toolbox for MATLAB
    def get_cam_data(self):
        if self.sim==1:
            kx=0.008/10**(-5)
            ky=0.008/10**(-5)
            u0=512
            v0=512
            cam = {'kx': kx, 'ky': ky, "u0": u0, "desiruvv0": v0}
            return cam
        f=open(self.califilename)
        yamldata=yaml.load(f)
        #print yamldata
        kx =  yamldata['camera_matrix']['data'][0]
        #print kx
        ky = yamldata['camera_matrix']['data'][4]
        #print ky
        u0=yamldata['camera_matrix']['data'][2]
        v0 = yamldata['camera_matrix']['data'][5]
        cam = {'kx': kx, 'ky': ky, "u0": u0, "v0": v0}
        return cam
        #print yaml.load(f)


    """ read data from yaml, here it temporary uses the list exist"""
    def get_instrinc_param(self):
        data = numpy.array(
            [895.957338, 0.000000, 333.621431, 0.000000, 895.348321, 241.448886, 0.000000, 0.000000, 1.000000])
        instrinc_param = data.reshape((3, 3))
       # print(instrinc_param)
        return instrinc_param

    """  input : camera pos of ar tag  3*1; only for one point-0.08401211423342386, 0.004883804261170381, 0.7855804355335336, -0.09810482217655597, 0.9939528146814213, -0.03307682330079316, -0.036594669187119074
        output :  image space u,v coordinate"""
    def get_uv_from_ar(self,pos):
        #pos = [-0.0694628511461, 0.0487799361822, 0.988924230718]
        # print("pos:", pos)
        cam_pos = numpy.array( pos )
        # 归一化
        # print("cam pos1:", cam_pos)rate = rospy.Rate(0.1)
        # cam_pos = cam_pos.reshape((3,1)) / cam_pos[2]
        cam_pos = cam_pos.T / cam_pos[2]
        # print(cam_pos)
        # print("cam pos2:", cam_pos)
        imgpos = numpy.dot( self.get_instrinc_param(), cam_pos)
        #print imgpos
        imgpos = imgpos[0:2]
        #print("imgps2:", imgpos)
        return imgpos.tolist()
        # print(imgpos)
    def get_z_from_ar(self):
        pass
    #cal image jacbian
    def vis2jac1(self,uv,z):
        cam=self.get_cam_data()
        rh0=[0.0000032,0.0000032]
        camf=0.6240429#m
        kx = cam['kx']
        ky = cam['ky']
        #--------------sgl-------------
        # print kx
        arfx=kx/camf
        arfy=ky/camf
        # kx=arfx*camf
        # ky=arfy*camf
        uba=uv[0]-cam['u0']
        vba=uv[1]-cam['v0']
        L=[[-arfx/z,0,uba/z,1/arfx*uba*vba,-(arfx**2+uba**2)/arfx,vba,0,-arfy/z,vba/z,(arfy**2+vba**2)/arfy,-uba*vba/arfx,-uba]]
        J=numpy.array(L).reshape((2,6))
        return J
    #uv more than one
    #,uv = [[672, 672], [632, 662]]
    def vis2jac(self,uv,z):
        cam=self.get_cam_data()
        rh0=[0.0000032,0.0000032]
        camf=0.6240429#m
        kx = cam['kx']
        ky = cam['ky']
        #--------------sgl-------------
        # print kx
        pu=camf/kx
        pv=camf/ky
        #f/pu=kx,f/pv=ky
        # arfx=kx/camf
        # arfy=ky/camf
        # kx=arfx*camf
        # ky=arfy*camf
        uba=uv[0]-cam['u0']
        vba=uv[1]-cam['v0']
        L=[[-kx/z,0,uba/z,1/kx*uba*vba,-(kx**2+uba**2)/kx,vba,0,-ky/z,vba/z,(ky**2+vba**2)/ky,-uba*vba/ky,-uba]]
        J=numpy.array(L).reshape((2,6))
        return J
    def vis2jac_mt1(self,uvm,z):
        if len(uvm)>1:
            L=self.vis2jac(uvm[0],z)
            #L=numpy.array(L).reshape((2,6))
            for i in xrange(1,len(uvm)):
                J=numpy.row_stack((L,self.vis2jac(uvm[i],z)))
                #print "-------",i,J
                L=J
            #print "vision jacobian last\n",J
            return J
        else:
            return self.vis2jac(uvm[0],z)

    def get_feature_error(self,desireuv,nowuv):
        kk=numpy.mat(nowuv).T-numpy.mat(desireuv).T
        #kk=numpy.mat([5,0])
        return kk.reshape((1,2))
    #cam speed (udot,vdot)(xdot,ydot,zdot,wxdot,wydot,wzdot)
    #get camera frame speed,you must change to ee frame
    def get_cam_vdot(self,uvm,z,desireuv,nowuv):
        J=self.vis2jac_mt1(uvm,z)
        JJ=numpy.linalg.pinv(J)
        e=self.get_feature_error(desireuv,nowuv)
        vdot=self.lambda1*numpy.dot(JJ,e.T)
        return vdot
    def get_joint_speed_1(self,armaker_pose,xd,yd,zd,q):
        #1,get base to ee jacabian
        Jacabian_joint,T_06=self.get_jacabian_from_joint(self.urdfname,q,0)
        #2,get ee(AX=XB) to camera frame jacabian
        # X=get_ur_X()#numpu array
        XX=[0, 0, 1,1, 0, -1, 0,1, 1, 0, 0,1,0,0,0,1]
        
        X=numpy.matrix(XX).reshape((4,4))
        ebT=T_06
        #tr2jac
        jac = tr2jac(X,1)
        jac_b2e=tr2jac(T_06,0)
        print "------X",X
        inv_X_jac = jac.I
        lamda1=-1
        #get ee speed
        #print "tr2jac-----\n",jac
        # cam_speed = self.get_cam_vdot(uvm, z, desireuv, nowuv)
        # print "cam_speed--------",cam_speed
        detas=self.get_feature_error_xyz(armaker_pose,xd,yd,zd)
        L=[-1,0,0,0,-1,0,0,0,-1]
        print "numpy.matrix(L).I",numpy.matrix(L).reshape((3,3)).I
        print "numpy.matrix(detas).T",numpy.matrix(detas).T

        Vc=-lamda1*numpy.dot(numpy.matrix(L).reshape((3,3)).I,numpy.matrix(detas).T)

        print "Vc",Vc.tolist()
        Vcc=[Vc.tolist()[0][0],Vc.tolist()[1][0],Vc.tolist()[2][0],0,0,0]

        ee_speed_in_eeframe = np.dot(inv_X_jac, numpy.matrix(Vcc).T)
        v_list = ee_speed_in_eeframe.reshape((1, 6)).tolist()[0]
        #[z,y,]
        flag_list = [1, 1, 1, 1, 1, 1]
        vdot_z = [1.0 * v_list[i] * flag_list[i] for i in range(6)]
        ee_speed_in_base = np.dot(jac_b2e.I, numpy.mat(vdot_z).T)
        print "ee_speed-----before changing--------",ee_speed_in_base

        print("ee_speed_after--------------\n",vdot_z)
        j_speed=numpy.dot(Jacabian_joint.I,ee_speed_in_base)
        return j_speed
    def get_feature_error_xyz(self,armaker_pose,xd,yd,zd):#armaker---x,y,z
        return [armaker_pose[0]-xd,(armaker_pose[1]-yd)*1,(armaker_pose[2]-zd)*-1]

    def get_joint_speed(self,armaker_pose,xd,yd,zd,q,xdot,ydot,n,f,fd,lamdaf):#f=[],fd=[],lamdaf=[]
        #1,get base to ee jacabian
        Jacabian_joint,T_06=self.get_jacabian_from_joint(self.urdfname,q,0)
        #2,get ee(AX=XB) to camera frame jacabian
        # X=get_ur_X()#numpu array
        XX=[0, 0, 1,1, 0, -1, 0,1, 1, 0, 0,1,0,0,0,1]
        X=numpy.matrix(XX).reshape((4,4))
        ebT=T_06
        #tr2jac
        jac = tr2jac(X,1)
        jac_b2e=tr2jac(T_06,0)
        print "------X",X
        inv_X_jac = jac.I
        lamdas1=[-0.5,0,0,0,-0.5,0,0,0,-0.01]
        # lamda1=-0.1
        #get ee speed
        #print "tr2jac-----\n",jac
        # cam_speed = self.get_cam_vdot(uvm, z, desireuv, nowuv)
        # print "cam_speed--------",cam_speed


        detaf = [f[0]-fd[0],f[1]-fd[1],f[2]-fd[2]]#[ (f - fd)]
        fz=f[2]
        fdz=fd[2]
        fx=f[0]
        fdx=fd[0]
        fy=f[1]
        fdy=fd[1]
        lamdaf_matrix=[lamdaf[0],0,0,0,lamdaf[1],0,0,0,lamdaf[2]]
        fzpie = fz - self.netf_zero[2]#f'
        fzdpie = fdz - self.netf_zero[2]

        fxpie = fx - self.netf_zero[0]#f'
        fxdpie = fdx - self.netf_zero[0]

        fypie = fy - self.netf_zero[1]#f'
        fydpie = fdy - self.netf_zero[1]

        z_detaff_pub=rospy.Publisher("/z_detaf_info", Float64, queue_size=10)
        z_fpie_pub = rospy.Publisher("/fz_pie_info", Float64, queue_size=10)
        z_fdpie_pub = rospy.Publisher("/fdz_pie_info", Float64, queue_size=10)
        x_detaff_pub=rospy.Publisher("/x_detaf_info", Float64, queue_size=10)
        x_fpie_pub = rospy.Publisher("/fx_pie_info", Float64, queue_size=10)
        x_fdpie_pub = rospy.Publisher("/fdx_pie_info", Float64, queue_size=10)
        y_detaff_pub=rospy.Publisher("/y_detaf_info", Float64, queue_size=10)
        y_fpie_pub = rospy.Publisher("/fy_pie_info", Float64, queue_size=10)
        y_fdpie_pub = rospy.Publisher("/fdy_pie_info", Float64, queue_size=10)

        z_detaffl_pub = rospy.Publisher("/z_detaf_L_info", Float64, queue_size=10)
        x_detaffl_pub = rospy.Publisher("/x_detaf_L_info", Float64, queue_size=10)
        y_detaffl_pub = rospy.Publisher("/y_detaf_L_info", Float64, queue_size=10)


        z_fpie_pub.publish(fzpie)
        z_fdpie_pub.publish(fzdpie)
        z_detaff_pub.publish(fz-fdz)

        x_fpie_pub.publish(fxpie)
        x_fdpie_pub.publish(fxdpie)
        x_detaff_pub.publish(fx-fdx)

        y_fpie_pub.publish(fypie)
        y_fdpie_pub.publish(fydpie)
        y_detaff_pub.publish(fy-fdy)

        z_detaffl_pub.publish(lamdaf[2]/lamdas1[8]*(fz-fdz))
        y_detaffl_pub.publish(lamdaf[1] / lamdas1[4] * (fy - fdy))
        x_detaffl_pub.publish(lamdaf[0] / lamdas1[0] * (fx - fdx))


        detas=self.get_feature_error_xyz(armaker_pose,xd,yd,zd)
        L=[-1,0,0,0,-1,0,0,0,-1]
        print "numpy.matrix(L).I",numpy.matrix(L).reshape((3,3)).I
        print "numpy.matrix(detas).T",numpy.matrix(detas).T
        sddot=[xdot,ydot,0]
        Vc=numpy.dot(numpy.matrix(L).reshape((3,3)).I,-numpy.matrix(lamdas1).reshape((3,3))*numpy.matrix(detas).T+0*n*numpy.matrix(sddot).T+numpy.dot(numpy.matrix(lamdaf_matrix).reshape((3,3)),numpy.matrix(detaf).T))

        print "Vc",Vc.tolist()
        Vcc=[Vc.tolist()[0][0],Vc.tolist()[1][0],Vc.tolist()[2][0],0,0,0]

        ee_speed_in_eeframe = np.dot(inv_X_jac, numpy.matrix(Vcc).T)
        v_list = ee_speed_in_eeframe.reshape((1, 6)).tolist()[0]
        #[z,y,]
        flag_list = [1, 1, 1, 0, 0, 0]
        vdot_z = [1.0 * v_list[i] * flag_list[i] for i in range(6)]
        ee_speed_in_base = np.dot(jac_b2e.I, numpy.mat(vdot_z).T)
        print "ee_speed-----before changing--------",ee_speed_in_base

        print("ee_speed_after--------------\n",vdot_z)
        j_speed=numpy.dot(Jacabian_joint.I,ee_speed_in_base)
        return j_speed,Vcc
    #
    def get_deta_joint_angular(self,detat,armaker_pose,xd,yd,zd,q,xdot,ydot,n,f,fd,lamda2):
        j_speed,Vcc=self.get_joint_speed(armaker_pose,xd,yd,zd,q,xdot,ydot,n,f,fd,lamda2)
        #print j_speed
        joint_angular=float(detat)*numpy.array(j_speed)
        #print '-------joint_angular-----\n',joint_angular
        return joint_angular
    def get_joint_angular(self,qnow,detajoint):
        #result=[]
        listangular=[]
        for i in range(len(detajoint.tolist())):
            listangular.append(detajoint.tolist()[i][0]+qnow[i])
        print "list",detajoint.tolist()
        return listangular

def main():
    # urdfname="/data/ros/ur_ws_yue/src/ur5_planning/urdf/ur5.urdf"
    # filename="/data/ros/ur_ws_yue/src/ur5_planning/yaml/cam_500_industry.yaml"
    # urdfname="/data/ros/ur_ws/src/universal_robot/ur_description/urdf/ur5.urdf"

    urdfname="/data/ros/yue_ws_201903/src/visionbased_polishing/urdf/ur5.urdf"
    filename="/data/ros/yue_ws_201903/src/tcst_pkg/yaml/cam_300_industry_20200518.yaml"

    desiruv=[]
    # desiruv=[[168,169]]
    # lambda1=3.666666

    detat=0.05
    z=0.25
    ace=50
    vel=0.1
    urt=0
    ratet=5
    netf_zero = [0.0,0.0,0.0]#3.5#[]
    p0=VisonControl(filename,0,urdfname,netf_zero)

    # the original parameter is shown as follows
    # lamdaf=[-0.001/2,-0.001/2,-0.001/2]#

    lamdaf=[0.001/2,0.001/2,0.001/2]
    ur_reader = Urposition()
    ur_sub = rospy.Subscriber("/joint_states", JointState, ur_reader.callback)

    x_error_pub = rospy.Publisher("/feature_x_error", Float64, queue_size=10)
    y_error_pub = rospy.Publisher("/feature_y_error", Float64, queue_size=10)
    z_error_pub = rospy.Publisher("/feature_z_error", Float64, queue_size=10)
    z_depth_pub = rospy.Publisher("/camera_depth", Float64, queue_size=10)
    now_uv_pub = rospy.Publisher("/nowuv_info", uv, queue_size=10)
    ur_pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)

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

    z_impedance_error_pub = rospy.Publisher("/z_impedance_error_info", Float64, queue_size=10)
    y_impedance_error_pub = rospy.Publisher("/y_impedance_error_info", Float64, queue_size=10)
    x_impedance_error_pub = rospy.Publisher("/x_impedance_error_info", Float64, queue_size=10)


    rate = rospy.Rate(ratet)
    count=1
    marker_zero=[]
    while not rospy.is_shutdown():

        try:
            sturucture_point_xn=structure_xnynan.sturucture_point_xn_buf
            sturucture_point_yn=structure_xnynan.sturucture_point_yn_buf
            sturucture_point_an=structure_xnynan.sturucture_point_an_buf
            force_list = netf_reader.ave_netf_force_data
            if len(sturucture_point_xn)!=0 and len(sturucture_point_yn)!=0 and len(sturucture_point_an)!=0:
                q_now = ur_reader.ave_ur_pose
                # if len(xd_get.sturucture_point_xd_buf)!=0 and len(xd_get.sturucture_point_xd_buf)!=0:

                xdot = 0.01
                ydot = 0.01

                # xd = marker_zero[0]+xdot*count/30
                # print "marker_zero",marker_zero
                # print "xd---------",xd
                # xd=xd_get.sturucture_point_xd_buf[-1]
                xd=338
                print "xd=---------",xd
                # yd = 0.01
                # yd=xd_get.sturucture_point_yd_buf[-1]
                yd=255
                zd = 0.15
                xnynan=[sturucture_point_xn[-1],sturucture_point_yn[-1],sturucture_point_an[-1]]
                print "xnynan",xnynan
                print force_list
                if len(force_list) != 0 and len(tool_get.Ur_tool_velocity_buf)!=0:
                    print "force sensor z", force_list
                    netf=[force_list[0],force_list[1],force_list[2]]
                    # netf_z = force_list[2]
                    print "netf ", netf
                    # fd=[-27.5,-3.3,1.1]#-1.5#[]
                    fd=[-3.15999984741,-1.18000030518,-7.6]
                    # x: -3.15999984741
                    # y: -1.18000030518
                    # z: -7.6000061035
                    # lamda2=-0.001/2#(0.001)/5

                    joint_speed,vcc=p0.get_joint_speed(xnynan, xd, yd, zd, q_now, xdot, 0 * ydot, count, netf, fd, lamdaf)
                    print "joint_speed,vcc\n", joint_speed,vcc


                    # if len(tool_get.Ur_tool_velocity_buf)!=0:
                    z_impedance_error=tool_get.Ur_tool_velocity_buf[-1][2]-vcc[2]
                    y_impedance_error = tool_get.Ur_tool_velocity_buf[-1][1] - vcc[1]
                    x_impedance_error = tool_get.Ur_tool_velocity_buf[-1][0] - vcc[0]
                    z_impedance_error_pub.publish(z_impedance_error)
                    y_impedance_error_pub.publish(y_impedance_error)
                    x_impedance_error_pub.publish(x_impedance_error)
                    detas = p0.get_feature_error_xyz(xnynan, xd, yd, zd)

                    #get joint speed in ee frame

                    print "##############################################################"
                    print "deta joint angular---"
                    detaangular = p0.get_deta_joint_angular(detat, xnynan, xd, yd, zd, q_now, xdot, 0*ydot, count,netf,fd,lamdaf)
                    print detaangular
                    print "##############################################################"
                    count += 0.5

                print "count--------",count
                x_error_pub.publish(detas[0])
                y_error_pub.publish(detas[1])
                z_error_pub.publish(-1*detas[2])
                print "joint angular----"
                q_pub_now=p0.get_joint_angular(q_now,detaangular)
                print q_pub_now
                print "##############################################################"
                print "move ur base the servo system----"
                print "q_now\n", q_now
                print "q_pub_now\n",q_pub_now
                ss = "movej([" + str(q_pub_now[0]) + "," + str(q_pub_now[1]) + "," + str(q_pub_now[2]) + "," + str(
                    q_pub_now[3]) + "," + str(q_pub_now[4]) + "," + str(q_pub_now[5]) + "]," + "a=" + str(ace) + "," + "v=" + str(
                    vel) + "," + "t=" + str(urt) + ")"
                print ss
                ur_pub.publish(ss)
                rate.sleep()

            else:
                continue
        except:
            continue

if __name__=="__main__":
    main()