#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import time 
from std_msgs.msg import String,Float64,Int64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

import numpy as np
import numpy.matlib
from numpy.linalg import *
from frompitoangle import *
from math import *
from ur5_pose_get import *
class null_space_control:
    def __init__(self,q_start):
        rospy.init_node('null_space_test', anonymous=True)

        self.robot = URDF.from_xml_file("/data/ros/yue_ws/src/tcst_pkg/urdf/ur5.urdf")
        self.urscript_pub=rospy.Publisher("/ur_driver/URScript",String,queue_size=10)

        self.error_r_pub=rospy.Publisher("/null_space/error_r",Vector3,queue_size=10)
        self.rdes_pub=rospy.Publisher("/null_space/rdes",Vector3,queue_size=10)
        self.r_pub=rospy.Publisher("/null_space/r",Vector3,queue_size=10)
        self.condition_num_pub=rospy.Publisher("/null_space/condition_num",Float64,queue_size=10)

        self.q4_pub=rospy.Publisher("/null_space/q4",Float64,queue_size=10)
        self.q6_pub=rospy.Publisher("/null_space/q6",Float64,queue_size=10)


        self.aubo_q=q_start#[1.50040841e-03, -2.83640237e+00, 1.53798406e+00, 1.29841831e+00, 1.50040840e-03, 3.14159265e+00]
        # self.aubo_q=[0.4169788306196942, -1.3621199297826534, -2.011464437502717, -2.22014083451496, -1.5707963267948966, 1.1538174961752024]
        self.aubo_q1=np.matrix(self.aubo_q)
        
        self.vel=1.05
        self.ace=1.4
        self.t=0

        self.kpr=0.3
        self.kq4=1000
        self.kq6=1000 
        self.bq4=pi/20
        self.bq6=pi/20



    def kdl_computation(self,q):
        kdl_kin = KDLKinematics(self.robot, "base_link", "tool0")
        pose = kdl_kin.forward(q) 
        J = kdl_kin.jacobian(q)
        return pose, J



    def qdot_generation(self,q,arm_rdes,arm_rdot_des):
        # rospy.logerr("qdot ---q"+str(q))
        pose, J1=self.kdl_computation(q)
        rospy.logerr("pose"+str(pose))
        # if not (pose is None):
        #     return

            
        J2_1=np.matrix([[J1[0,1],J1[0,2],J1[0,3],J1[0,5]]])
        J2_2=np.matrix([[J1[2,1],J1[2,2],J1[2,3],J1[2,5]]])
        J=np.vstack((J2_1,J2_2))
        print("J is:",J)
        # rospy.logerr("Det J"+str(det(J)))
        

        position_x=pose[0,3]
        position_y=pose[1,3]
        position_z=pose[2,3]
        tran_mat=np.matrix([[position_x],[position_y],[position_z]])

        delta_rdot=arm_rdot_des-self.kpr*(tran_mat-arm_rdes)

        print("arm_rdot_des is", arm_rdot_des)
        print("tran_mat is",tran_mat)
        print("arm_rdes is",arm_rdes)
        print("delta_rdot is", delta_rdot)

        print("the combined J is:",np.dot(J,J.T))

        # pJ=np.dot(J.T,np.linalg.pinv(np.dot(J,J.T)))

        
        print("np.linalg.pinv",np.linalg.pinv(J))
        pJ=np.linalg.pinv(J)
        print("pJ is",pJ)
        null_mat=np.matlib.identity(4,dtype=float)-np.dot(pJ,J)

        fq4=q[2]**2-self.bq4**2
        print("fq4 is:",fq4)

        q4=self.kq4*min(0,fq4)
        print("q4 is:",q4)
        fq6=q[5]**2-self.bq6**2

        print("fq6 is:",fq6)
        q6=self.kq6*min(0,fq6)
        print("q6 is:",q6)
        self.q4_pub.publish(q4)
        self.q6_pub.publish(q6)

        delta_qdot=np.matrix([0.0,q4,0.0,q6])

        delta_rdot1=np.matrix([[delta_rdot[0,0]],[delta_rdot[2,0]]])


        print("delta_rdot1",delta_rdot1)
        print("np.dot(null_mat,delta_qdot.T)",np.dot(null_mat,delta_qdot.T))
        qdot1=np.dot(pJ,delta_rdot1)+np.dot(null_mat,delta_qdot.T)

        rospy.logerr("qdot1"+str(qdot1))
        qdot=np.array([0.0,qdot1[0,0],qdot1[1,0],qdot1[2,0],0.0,qdot1[3,0]])


        print("qdot is:",qdot)
        mat=np.dot(J,null_mat)
        print("the mat is: ",mat)

        condition_num=np.linalg.det(np.dot(J,J.T))
        self.condition_num_pub.publish(condition_num)

        tran_mat=pose[0:3,3]
        rdot=tran_mat-arm_rdes

        error_r=Vector3()
        error_r.x=rdot[0,0]
        error_r.y=rdot[1,0]
        error_r.z=rdot[2,0]
        rdes=Vector3()
        rdes.x=arm_rdes[0,0]
        rdes.y=arm_rdes[1,0]
        rdes.z=arm_rdes[2,0]
        r=Vector3()
        r.x=tran_mat[0,0]
        r.y=tran_mat[1,0]
        r.z=tran_mat[2,0] 

        self.error_r_pub.publish(error_r)
        self.rdes_pub.publish(rdes)
        self.r_pub.publish(r)

        return qdot

    def moveur(self,q):
        ss="movej(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(self.ace)+","+"v="+str(self.vel)+","+"t="+str(self.t)+")"
        self.urscript_pub.publish(ss)
    def urscript_speedj_pub(self,vel, ace,t):
        ss = "speedj([" + str(vel[0]) + "," + str(vel[1]) + "," + str(vel[2]) + "," + str(vel[3]) + "," + str(
            vel[4]) + "," + str(vel[5]) + "]," + "a=" + str(ace) + "," + "t=" + str(t) + ")"
        self.urscript_pub.publish(ss)
def getpi(listb):
    lista=[]
    listcc=[]
    for i in listb:
        temp=i/180*3.14
        lista.append((temp,i))
        listcc.append(temp)
    return listcc
def main():
    step=0.01
    time1=10
    tnum=int(time1/step+1)
    omega=0.5
    radius=0.15

    flag=1
    q_start=[44.87,-70.67,40.06,299.46,-89.69,175.71]
    start_rad=getpi(q_start)
    null_space_o=null_space_control(start_rad)

    count=0
    ur_reader = Urposition()
    ur_sub = rospy.Subscriber("/joint_states", JointState, ur_reader.callback)
    
    pose, J1=null_space_o.kdl_computation(start_rad)

    position_x=pose[0,3]
    position_z=pose[2,3]

    ratet=10
    urt=1.0/ratet
    ace=10
    rate = rospy.Rate(ratet)
    while not rospy.is_shutdown():
        
        q_now=ur_reader.ave_ur_pose
        if len(q_now)!=0:
            t=step*count
            arm_rdes=np.matrix([radius*cos(omega*t)+position_x-radius, 0.48, radius*sin(omega*t)+position_z])
            arm_rdes=arm_rdes.T
            arm_rdot_des=np.matrix([-radius*omega*sin(omega*t), 0.0, radius*omega*cos(omega*t)])
            arm_rdot_des=arm_rdot_des.T

            arm_qdot=null_space_o.qdot_generation(q_now,arm_rdes,arm_rdot_des)

            rospy.logerr("arm_qdot---------"+str(arm_qdot))

            # null_space_o.urscript_speedj_pub(arm_qdot,ace,urt)

            rate.sleep()
            count+=1
            # if count>1000:
            #     break
if __name__ == "__main__":
    main()