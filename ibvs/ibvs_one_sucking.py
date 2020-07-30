#!/usr/bin/env python
# -*- coding: utf-8 -*-

import frompitoangle
import numpy
from numpy import matlib,linalg
#Import the module
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
from ur5_kinematics import Kinematic
#import cv2
import rospy
import yaml,os
from trans_methods import *


from hand_in_eye import *
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from ur5_pose_get import *
from std_msgs.msg import Float64
import re,os
class VisonControl():
    def __init__(self,califilename,sim,lambda1,urdfname):
        self.califilename=califilename
        self.sim=sim
        self.lambda1=lambda1
        self.urdfname=urdfname
        
        rospy.init_node("vision_control_one_feature")
        self.sub_object_uv=rospy.Subscriber("/object_data_pub", String, self.object_callback)
        self.uv_list_buffer=[]
    #if flag=1,use our kinematics for inverse
    def object_callback(self,msg):
        tupletemp = re.findall(r'\-?\d+\.?\d*', msg.data)
        print(int(tupletemp[0]),int(tupletemp[1]))
        print("##################################")
        if len(tupletemp)!=0:
            if len(self.uv_list_buffer)>10:
                self.uv_list_buffer=self.uv_list_buffer[1:]    
                self.uv_list_buffer.append([int(tupletemp[0]),int(tupletemp[1])])
            else:
                self.uv_list_buffer.append([int(tupletemp[0]),int(tupletemp[1])])
    def get_jacabian_from_joint(self,urdfname,q,flag):
        #robot = URDF.from_xml_file("/data/ros/ur_ws/src/universal_robot/ur_description/urdf/ur5.urdf")
        robot = URDF.from_xml_file(urdfname)
        tree = kdl_tree_from_urdf_model(robot)
        # print tree.getNrOfSegments()
        chain = tree.getChain("base_link", "tool0")
        # print chain.getNrOfJoints()
        # forwawrd kinematics
        kdl_kin = KDLKinematics(robot, "base_link", "tool0")
        # print(q)
        #q = [0, 0, 1, 0, 1, 0]
        pose = kdl_kin.forward(q)  # forward kinematics (returns homogeneous 4x4 matrix)
        print "pose-----",pose
        # print(pose)
        #print list(pose)
        # q0=Kinematic()
        # print(q0.Forward(q))
        # if flag==1:
        #     q_ik=q0.best_sol_for_other_py( [1.] * 6, 0, q0.Forward(q))
        # else:
        #     q_ik = kdl_kin.inverse(pose)  # inverse kinematics
        # print "----------iverse-------------------\n", q_ik

        # if q_ik is not None:
        #     pose_sol = kdl_kin.forward(q_ik)  # should equal pose
        #     print "------------------forward ------------------\n",pose_sol
        J = kdl_kin.jacobian(q)
        print('kdl J:', J)
        return J,pose
    #kx=f/px,ky=f/py
    #sim=1,use camera default from Macine Vision Toolbox for MATLAB
    def get_cam_data(self):
    
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

    #cal image jacbian
    def vis2jac(self,uv,z):
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
    #只需要xy,z轴旋转
    def get_cam_vdot_wz(self, uvm, z, desireuv,nowuv):
        J = self.vis2jac_mt1(uvm, z)
        print "J:", J
        JJ = numpy.linalg.pinv(J)  # pseduo inverse of jacobian

        feature_error = self.get_feature_error( desireuv, nowuv )
        # print "e:", feature_error
        print "JJ:", JJ
        vdot = self.lambda1 * np.dot(JJ, feature_error.T)
        print "vdot:", vdot
        v_list = vdot.reshape((1, 6)).tolist()[0]
        flag_list = [1, 0, -1, 1, 1, 1]  # [z,x,y,wx,wz,wy ]
        vdot_z = [1.0 * v_list[i] * flag_list[i] for i in range(6)]
        # vdot_z = v_list[:2] + [0, 0, 0]
        # vdot_z.append( v_list[-1] )
        print "vdot_z:", vdot_z
        return np.matrix(vdot_z).T
    #samebody tranlasition to jacbian
    #joint speed (q0dot,q1dot,q2dot,q3dot,q4dot,q5dot)
    def get_ur_X(self,):
        ar_info = [
            # translation: 
            0.0778746546832,
            0.000796420186253,
            -0.0853968073946,
            # rotation: 
            -0.716503195961,
            0.00435505650847,
            -0.694984883822,
            0.0600017909651

        ]
        aa=get_X_from_ar_quaternion(ar_info)
        aa=np.mat(aa)
        # print "X",aa.reshape((4,4))
        # return aa.reshape((4, 4))
        X=numpy.matrix([[0.0,1.0,0.0,0.0],[-1.0,0.0,0.0,+0.12],[0.0,0.0,1.0,+0.09],[0.0,0.0,0.0,1.0]])
        return X
    def get_joint_speed(self,uvm,z,desireuv,nowuv,q):
        #1,get base to ee jacabian
        Jacabian_joint,T_06=self.get_jacabian_from_joint(self.urdfname,q,0)
        print(q)
        #2,get ee(AX=XB) to camera frame jacabian
        X=self.get_ur_X()#numpu array
        ebT=T_06
        #tr2jac
        jac = tr2jac(X,1)

        jac_b2e=tr2jac(T_06,0)
        #print "------X",X
        inv_X_jac = jac.I
        #get ee speed
        #print "tr2jac-----\n",jac
        cam_speed = self.get_cam_vdot(uvm, z, desireuv, nowuv)
        print "cam_speed--------",cam_speed
        ee_speed_in_eeframe = np.dot(inv_X_jac, cam_speed)
        v_list = ee_speed_in_eeframe.reshape((1, 6)).tolist()[0]
        #[z,y,]
        # flag_list = [0, 1, 1, 0, 0, 0]
        flag_list = [1, 1, 1, 0, 0, 0]
        vdot_z = [1.0 * v_list[i] * flag_list[i] for i in range(6)]
        ee_speed_in_base = np.dot(jac_b2e.I, numpy.mat(vdot_z).T)
        print "ee_speed-----before changing--------",ee_speed_in_base

        print("ee_speed_after--------------\n",vdot_z)
        j_speed=numpy.dot(Jacabian_joint.I,ee_speed_in_base)
        return j_speed
    #
    def get_deta_joint_angular(self,detat,uvm,z,desireuv,nowuv,q):
        j_speed=self.get_joint_speed(uvm,z,desireuv,nowuv,q)
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
    def return_error_ok(self,feature_error_x,feature_error_y):
        if abs(feature_error_x) <=2 and abs(feature_error_y)<=2:
            return True
        else:
            return False
    def get_inverse_to_box(self,pub_now,x_length,y_length,depth):
        q0=Kinematic()
        weights=[1.] * 6
        T0=q0.Forward(pub_now)
        print(T0)
        temp=[]

        for i in range(len(T0)):
            if i==3:
                temp.append(T0[3]+x_length)
            elif i==7:
                temp.append(T0[7]+y_length)
            elif i==11:
                temp.append(T0[11]+depth)
            else:
                temp.append(T0[i])
        print("temp",temp)
        kk=q0.best_sol(weights,pub_now,temp)
        return kk
    def move_ur(self,ur_pub,q_pub_now,vel,ace,urt):

        ss = "movej([" + str(q_pub_now[0]) + "," + str(q_pub_now[1]) + "," + str(q_pub_now[2]) + "," + str(q_pub_now[3]) + "," + str(q_pub_now[4]) + "," + str(q_pub_now[5]) + "]," + "a=" + str(ace) + "," + "v=" + str(vel) + "," + "t=" + str(urt) + ")"
        print ss
        ur_pub.publish(ss)
    def move_to_sucking(self,ur_pub,pub_now,x_length,y_length,depth,vel,ace,urt):
        if len(pub_now)!=0:
            q_pub_now=self.get_inverse_to_box(pub_now,x_length,y_length,depth)
            print("move q pub now",q_pub_now)
            ss = "movej([" + str(q_pub_now[0]) + "," + str(q_pub_now[1]) + "," + str(q_pub_now[2]) + "," + str(q_pub_now[3]) + "," + str(q_pub_now[4]) + "," + str(q_pub_now[5]) + "]," + "a=" + str(ace) + "," + "v=" + str(vel) + "," + "t=" + str(urt) + ")"
            print ss
            ur_pub.publish(ss)
    def getpi(sefl,listb):
        lista=[]
        listcc=[]
        for i in listb:
            temp=i/180*3.14
            lista.append((temp,i))
            listcc.append(temp)
        return listcc
def main():
    urdfname="/data/ros/yue_ws/src/tcst_pkg/urdf/ur5.urdf"
    filename="/data/ros/yue_ws/src/tcst_pkg/yaml/cam_300_industry_20200518.yaml"
    # urdfname="/data/ros/ur_ws/src/universal_robot/ur_description/urdf/ur5.urdf"
    desiruv=[]
    # desiruv=[[230,290]]
    lambda1=-2.666666
    detat=0.05
    z=0.92
    ace=50
    vel=0.1
    urt=0
    ratet=10
    p0=VisonControl(filename,0,lambda1,urdfname)

    ur_reader = Urposition()
    ur_sub = rospy.Subscriber("/joint_states", JointState, ur_reader.callback)

    u_error_pub = rospy.Publisher("/feature_u_error", Float64, queue_size=10)
    v_error_pub = rospy.Publisher("/feature_v_error", Float64, queue_size=10)
    z_depth_pub = rospy.Publisher("/camera_depth", Float64, queue_size=10)

    ur_pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)

    down_to_q=[]
    desire_joint_angular=[124.08,-86.65,60.29,295.00,-89.38,166.73]
    start_angular=[44.87,-70.67,40.06,299.46,-89.69,175.71]
    cartisian_feedback=p0.getpi(desire_joint_angular)
    start_angular_back=p0.getpi(start_angular)
    rate = rospy.Rate(ratet)
    while not rospy.is_shutdown():
        # print "haha"
        desiruv=[]
        uvlist=[]
        open_ibvs_flag=rospy.get_param("open_ibvs_flag")
        open_go_desire_flag=rospy.get_param("open_go_desire_flag")
        open_go_to_desire=rospy.get_param("open_go_to_desire")
        # print(p0.uv_list_buffer)
        if open_ibvs_flag==1:
            if len(p0.uv_list_buffer)!=0:

                uvlist.append(p0.uv_list_buffer[-1])
                # uvlist.append([313,212])
                print "##############################################################"
                print "uv-list------\n",uvlist
                print "###########################################################"

                desiruv.append([333,241])
                # desiruv.append([550,339])

                #get error
                print "##############################################################"
                feature_error=p0.get_feature_error(desiruv,uvlist[0])
                print "Ibvs is ok?---",p0.return_error_ok(feature_error.tolist()[0][0],feature_error.tolist()[0][1])
                if p0.return_error_ok(feature_error.tolist()[0][0],feature_error.tolist()[0][1])==False:
                    print "feature error\n",feature_error
                    u_error_pub.publish(feature_error.tolist()[0][0])
                    v_error_pub.publish(feature_error.tolist()[0][1])
                    print "##############################################################"

                    print "camera vdot\n",p0.get_cam_vdot_wz(uvlist,z,desiruv,uvlist[0])
                    print "##############################################################"
                    q_now=ur_reader.ave_ur_pose
                    #get joint speed in ee frame
                    print "##############################################################"
                    print "q_now\n", q_now
                    print "joint speed\n",p0.get_joint_speed(uvlist,z,desiruv,uvlist[0],q_now)
                    print "deta joint angular---"
                    detaangular=p0.get_deta_joint_angular(detat,uvlist, z, desiruv, uvlist[0], q_now)
                    print detaangular
                    print "##############################################################"
                    print "joint angular----"
                    q_pub_now=p0.get_joint_angular(q_now,detaangular)
                    print q_pub_now
                    print "##############################################################"
                    print "move ur base the servo system----"
                    print "q_now\n", q_now
                    print "q_pub_now\n",q_pub_now
                    down_to_q=q_pub_now
                    ss = "movej([" + str(q_pub_now[0]) + "," + str(q_pub_now[1]) + "," + str(q_pub_now[2]) + "," + str(
                        q_pub_now[3]) + "," + str(q_pub_now[4]) + "," + str(q_pub_now[5]) + "]," + "a=" + str(ace) + "," + "v=" + str(
                        vel) + "," + "t=" + str(urt) + ")"
                    print ss
                    ur_pub.publish(ss)
                    #cable is x negtive
                if p0.return_error_ok(feature_error.tolist()[0][0],feature_error.tolist()[0][1])==True:
                    rospy.set_param("/open_go_to_object",0)
                    x_length=+0.112
                    y_length=+0.04#-0.005
                    z_depth=-0.31
                    # down_to_q=[0.6840241781464097, -1.7849443418136726, -1.2719627004551626, -1.671977857427553, 1.6020880964542237, 2.9842358676782488]
                    if len(down_to_q)!=0:
                        p0.move_to_sucking(ur_pub,down_to_q,x_length,y_length,z_depth,0.4,ace,urt)
                        time.sleep(4)
                        # os.system("rostopic pub /io_state std_msgs/String '55C8010155' -1")
                        p0.move_ur(ur_pub,down_to_q,0.4,ace,urt)
                        time.sleep(3)
                        # p0.move_ur(ur_pub,cartisian_feedback,0.4,ace,urt)
                        # time.sleep(5)
                        # rospy.set_param("open_ibvs_flag",0)
                        # rospy.set_param("open_go_desire_flag",1)
                        # rospy.set_param("/open_go_to_desire",1)

        # if open_go_desire_flag==1 and open_ibvs_flag==0:
        #     p0.uv_list_buffer=[]
        #     rospy.set_param("open_ibvs_flag",1)
        #     rospy.set_param("open_go_desire_flag",0)
        #     rospy.set_param("/open_go_to_object",1)
        #     rospy.set_param("/open_go_to_desire",0)
        #     x_length=0
        #     y_length=0
        #     z_depth=-0.58
        #     p0.move_to_sucking(ur_pub,cartisian_feedback,x_length,y_length,z_depth,0.4,ace,urt)
        #     time.sleep(2)
        #     os.system("rostopic pub /io_state std_msgs/String '55C8010055' -1")
        #     p0.move_ur(ur_pub,cartisian_feedback,0.4,ace,urt)
        #     time.sleep(3)
        #     p0.move_ur(ur_pub,start_angular_back,0.4,ace,urt)
        #     time.sleep(5)
            



        rate.sleep()


if __name__=="__main__":
    main()