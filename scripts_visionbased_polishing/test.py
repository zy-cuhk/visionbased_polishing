import sys

import os

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
from math import *
from visionbased_polishing.msg import uv

o_path="/data/ros/yue_ws_201903/src/visionbased_polishing"
sys.path.append(o_path) 

import scripts_arm.frompitoangle
from scripts_arm.ur5_kinematics import Kinematic
from scripts_arm.trans_methods import *
from scripts_arm.ur5_pose_get import *
from scripts_arm.uv_sub_node import *
from scripts_arm.impedance_netf_data_get import *
from scripts_arm.ur_tool_velocity_sub import *

# o_path = os.getcwd() 
o_path="/data/ros/yue_ws_201903/src/visionbased_polishing"
print(o_path)
sys.path.append(o_path) 

# from scripts_arm.frompitoangle import *

Q9=[1.4794633333333334, -2.17445, -1.3624111111111112, 1.7372922222222222, -1.6854822222222223, 1.5698255555555556]
# display(getangle(Q9))



X=numpy.matrix([[0.0,1.0,0.0,0.0],[-1.0,0.0,0.0,+0.12],[0.0,0.0,1.0,+0.09],[0.0,0.0,0.0,1.0]])
# print("X is",X)
jac = tr2jac(X,1)
print("jac is:",jac)
inv_X_jac = jac.I
print(inv_X_jac)

a=numpy.matrix([[ 2.84977612e-04],
 [-2.59708888e-03],
 [ 2.89552529e-03],
 [-2.96031651e-04],
 [-2.84251602e-04],
 [ 2.04706758e-05]])

print(a[0,0])
print(a[1,0])