#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy,math
import Quaternion as Q
import time
from numpy import linalg

import yaml
import os
import tf
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
from ar_track_alvar_msgs.msg import AlvarMarkers

from sensor_msgs.msg import JointState
from ur5_pose_get import *
from std_msgs.msg import UInt16,Float64

from std_msgs.msg import String
from Functions_for_other_py import *
from get_arpose_from_ar import *


from std_msgs.msg import UInt8
class Hand_In_Eye_Calibration():
    def __init__(self,):
        pass
    def tsai(self,):
        pass
def main():
    # https://blog.csdn.net/yaked/article/details/77161160?utm_medium=distribute.pc_relevant_right.none-task-blog-BlogCommendFromMachineLearnPai2-3.nonecase&depth_1-utm_source=distribute.pc_relevant_right.none-task-blog-BlogCommendFromMachineLearnPai2-3.nonecase
    pass
if __name__ == "__main__":
    pass
