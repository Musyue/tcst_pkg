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

import rospy
import yaml,os
from trans_methods import *

import Quaternion as Q
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from ur5_pose_get import *
from std_msgs.msg import Float64
import re

class IBVSControl():
    def __init__(self,califilename,lambda1,urdfname):
        self.califilename=califilename
        self.lambda1=lambda1
        self.urdfname=urdfname
        
        rospy.init_node("ibvs_box_picking")
        self.sub_object_uv=rospy.Subscriber("/object_data_pub", String, self.object_callback)
        self.sub_desire_uv=rospy.Subscriber("/desire_data_pub", String, self.desire_callback)
        self.uv_list_buffer=[]
        self.uv_desire_list_buffer=[]

    def object_callback(self,msg):
        tupletemp = re.findall(r'\-?\d+\.?\d*', msg.data)
        print("Object callback data",int(tupletemp[0]),int(tupletemp[1]))
        print("##################################")
        if len(tupletemp)!=0:
            if len(self.uv_list_buffer)>10:
                self.uv_list_buffer=self.uv_list_buffer[1:]    
                self.uv_list_buffer.append([int(tupletemp[0]),int(tupletemp[1])])
            else:
                self.uv_list_buffer.append([int(tupletemp[0]),int(tupletemp[1])])
    def desire_callback(self,msg):
        tupletemp = re.findall(r'\-?\d+\.?\d*', msg.data)
        print("Desire callback data",int(tupletemp[0]),int(tupletemp[1]))
        print("##################################")
        if len(tupletemp)!=0:
            if len(self.uv_desire_list_buffer)>10:
                self.uv_desire_list_buffer=self.uv_desire_list_buffer[1:]    
                self.uv_desire_list_buffer.append([int(tupletemp[0]),int(tupletemp[1])])
            else:
                self.uv_desire_list_buffer.append([int(tupletemp[0]),int(tupletemp[1])])
    def get_jacabian_from_joint(self,urdfname,jointq,flag):
        #robot = URDF.from_xml_file("/data/ros/ur_ws/src/universal_robot/ur_description/urdf/ur5.urdf")
        robot = URDF.from_xml_file(urdfname)
        tree = kdl_tree_from_urdf_model(robot)
        # print tree.getNrOfSegments()
        chain = tree.getChain("base_link", "tool0")
        # print chain.getNrOfJoints()
        # forwawrd kinematics
        kdl_kin = KDLKinematics(robot, "base_link", "tool0")
        q=jointq
        #q = [0, 0, 1, 0, 1, 0]
        pose = kdl_kin.forward(q)  # forward kinematics (returns homogeneous 4x4 matrix)

        J = kdl_kin.jacobian(q)
        #print 'J:', J
        return J,pose

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
        arfx=kx/camf
        arfy=ky/camf
        # kx=arfx*camf
        # ky=arfy*camf
        uba=uv[0]-cam['u0']
        vba=uv[1]-cam['v0']
        L=[[-arfx/z,0,uba/z,1/arfx*uba*vba,-(arfx**2+uba**2)/arfx,vba,0,-arfy/z,vba/z,(arfy**2+vba**2)/arfy,-uba*vba/arfx,-uba]]
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

    def get_joint_speed(self,uvm,z,desireuv,nowuv,q):
        #1,get base to ee jacabian
        Jacabian_joint,T_06=self.get_jacabian_from_joint(self.urdfname,q,0)
        #2,get ee(AX=XB) to camera frame jacabian
        X=X=numpy.matrix([[0.0,1.0,0.0,0.0],[-1.0,0.0,0.0,+0.12],[0.0,0.0,1.0,+0.098],[0.0,0.0,0.0,1.0]])
        ebT=T_06
        #tr2jac
        jac = tr2jac(X,1)
        jac_b2e=tr2jac(T_06,0)
        #print "------X",X
        inv_X_jac = jac.I
        #get ee speed
        #print "tr2jac-----\n",jac
        cam_speed = self.get_cam_vdot(uvm, z, desireuv, nowuv)
        print("cam_speed--------",cam_speed)
        ee_speed_in_eeframe = np.dot(inv_X_jac, cam_speed)
        v_list = ee_speed_in_eeframe.reshape((1, 6)).tolist()[0]
        #[z,y,]
        flag_list = [1, 1, 0, 0, 0, 0]
        vdot_z = [1.0 * v_list[i] * flag_list[i] for i in range(6)]
        ee_speed_in_base = np.dot(jac_b2e.I, numpy.mat(vdot_z).T)
        # print("ee_speed-----before changing--------",ee_speed_in_base)

        # print("ee_speed_after--------------\n",vdot_z)
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
        # print "list",detajoint.tolist()
        return listangular
    def return_error_ok(self,feature_error_x,feature_error_y):
        if abs(feature_error_x) <=5 and abs(feature_error_y)<=5:
            return True
        else:
            return False
    def return_error_ok_desire(self,feature_error_x,feature_error_y):
        if abs(feature_error_x) <=5 and abs(feature_error_y)<=5:
            return True
        else:
            return False
    def get_inverse_to_box(self,pub_now,x_length,y_length,depth):
        q0=Kinematic()
        weights=[1.] * 6
        T0=q0.Forward(pub_now)
        # print(T0)
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
        # print("temp",temp)
        kk=q0.best_sol(weights,pub_now,temp)
        return kk
    def move_ur(self,ur_pub,q_pub_now,vel,ace,urt):
        ss = "movej([" + str(q_pub_now[0]) + "," + str(q_pub_now[1]) + "," + str(q_pub_now[2]) + "," + str(q_pub_now[3]) + "," + str(q_pub_now[4]) + "," + str(q_pub_now[5]) + "]," + "a=" + str(ace) + "," + "v=" + str(vel) + "," + "t=" + str(urt) + ")"
        rospy.loginfo(ss)
        ur_pub.publish(ss)
    def move_ur_l(self,ur_pub,pub_now,ace,vel,t):
        q=pub_now
        ss="movel(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(ace)+","+"v="+str(vel)+","+"t="+str(t)+")"
        rospy.loginfo(ss)
        ur_pub.publish(ss)   
    def move_to_sucking(self,ur_pub,pub_now,x_length,y_length,depth,vel,ace,urt):
        if len(pub_now)!=0:
            q_pub_now=self.get_inverse_to_box(pub_now,x_length,y_length,depth)
            self.move_ur_l(ur_pub,q_pub_now,vel,ace,urt)
    def getpi(self,listb):
        lista=[]
        listcc=[]
        for i in listb:
            temp=i/180*3.14
            lista.append((temp,i))
            listcc.append(temp)
        return listcc
    def caculate_place_pose_from_ref(self,pub_now,flag):
        final={}
        if flag:
            x_length=0.035
            y_length=0.035
        else:
            x_length=0.058
            y_length=0.058
        depth=0
        pos0=self.get_inverse_to_box(pub_now,x_length,y_length,depth)
        final.update({0:pos0})
        
        x_length=-0.08
        y_length=0.08
        depth=0
        pos1=self.get_inverse_to_box(pos0,x_length,y_length,depth)
        final.update({1:pos1})

        x_length=0.099
        y_length=0.099
        depth=0
        pos2=self.get_inverse_to_box(pos1,x_length,y_length,depth)
        final.update({2:pos2})

        x_length=0.0899
        y_length=0.0899
        depth=0
        pos3=self.get_inverse_to_box(pos2,x_length,y_length,depth)
        final.update({3:pos3})
        x_length=0.066
        y_length=-0.066
        depth=0
        pos4=self.get_inverse_to_box(pos1,x_length,y_length,depth)
        final.update({4:pos4}) 

        x_length=0.0699
        y_length=-0.0699
        depth=0
        pos5=self.get_inverse_to_box(pos4,x_length,y_length,depth)
        final.update({5:pos5})   

        x_length=0.01
        y_length=-0.01
        depth=0
        pos6=self.get_inverse_to_box(pos5,x_length,y_length,depth)
        final.update({6:pos6}) 
        x_length=0.098
        y_length=-0.098
        depth=0
        pos7=self.get_inverse_to_box(pos6,x_length,y_length,depth)
        final.update({7:pos7})
        x_length=0.098
        y_length=-0.098
        depth=0
        pos8=self.get_inverse_to_box(pos3,x_length,y_length,depth)
        final.update({8:pos8}) 
        x_length=0.098
        y_length=-0.098
        depth=0
        pos9=self.get_inverse_to_box(pos8,x_length,y_length,depth)
        final.update({9:pos9})
        return final
def main():
    urdfname="/data/ros/yue_ws/src/tcst_pkg/urdf/ur5.urdf"
    filename="/data/ros/yue_ws/src/tcst_pkg/yaml/cam_300_industry_20200518.yaml"
    # urdfname="/data/ros/ur_ws/src/universal_robot/ur_description/urdf/ur5.urdf"
    desiruv=[]
    lambda1=-1.166666
    detat=0.05
    z=0.92
    ace=50
    vel=0.1
    urt=0
    ratet=10
    p0=IBVSControl(filename,lambda1,urdfname)

    ur_reader = Urposition()
    ur_sub = rospy.Subscriber("/joint_states", JointState, ur_reader.callback)

    u_error_pub = rospy.Publisher("/feature_u_error", Float64, queue_size=10)
    v_error_pub = rospy.Publisher("/feature_v_error", Float64, queue_size=10)
    z_depth_pub = rospy.Publisher("/camera_depth", Float64, queue_size=10)

    ur_pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)

    down_to_q=[]
    desire_joint_angular=[124.08,-86.65,60.29,295.00,-89.38,166.73]
    start_angular=[44.87,-70.67,40.06,299.46,-89.69,182.71]
    cartisian_feedback=p0.getpi(desire_joint_angular)
    start_angular_back=p0.getpi(start_angular)

    rate = rospy.Rate(ratet)
    
    count_for_desire=0
    place_count=2
    while not rospy.is_shutdown():
        
        desiruv=[]
        uvlist=[]
        open_ibvs_flag=rospy.get_param("open_ibvs_flag")
        open_go_to_desire=rospy.get_param("open_go_to_desire")
        print("open_ibvs_flag,open_go_to_desire",open_ibvs_flag,open_go_to_desire)
        # print(p0.uv_list_buffer)
        if open_ibvs_flag==1:
            print("Object picking")
            if len(p0.uv_list_buffer)!=0:

                uvlist.append(p0.uv_list_buffer[-1])

                desiruv.append([333,241])

                feature_error=p0.get_feature_error(desiruv,uvlist[0])
                print "Ibvs is ok?---",p0.return_error_ok(feature_error.tolist()[0][0],feature_error.tolist()[0][1])
                if p0.return_error_ok(feature_error.tolist()[0][0],feature_error.tolist()[0][1])==False:
                    print "feature error\n",feature_error
                    u_error_pub.publish(feature_error.tolist()[0][0])
                    v_error_pub.publish(feature_error.tolist()[0][1])

                    q_now=ur_reader.ave_ur_pose
                    detaangular=p0.get_deta_joint_angular(detat,uvlist, z, desiruv, uvlist[0], q_now)

                    q_pub_now=p0.get_joint_angular(q_now,detaangular)

                    print "q_now\n", q_now
                    print "q_pub_now\n",q_pub_now
                    down_to_q=q_pub_now
                    ss = "movej([" + str(q_pub_now[0]) + "," + str(q_pub_now[1]) + "," + str(q_pub_now[2]) + "," + str(
                        q_pub_now[3]) + "," + str(q_pub_now[4]) + "," + str(q_pub_now[5]) + "]," + "a=" + str(ace) + "," + "v=" + str(
                        vel) + "," + "t=" + str(urt) + ")"
                    ur_pub.publish(ss)
                if p0.return_error_ok(feature_error.tolist()[0][0],feature_error.tolist()[0][1])==True:
                    rospy.set_param("/open_go_to_object",0)
                    # down_to_q=[0.6840241781464097, -1.7849443418136726, -1.2719627004551626, -1.671977857427553, 1.6020880964542237, 2.9842358676782488]
                    if len(down_to_q)!=0:
                        temp_q0=p0.get_inverse_to_box(down_to_q,0,0,-0.15)
                        p0.move_ur_l(ur_pub,temp_q0,0.2,ace,urt)
                        time.sleep(1) 
                        x_length=+0.112
                        y_length=+0.04#-0.005
                        z_depth=0
                        temp_q1=p0.get_inverse_to_box(temp_q0,x_length,y_length,z_depth)
                        p0.move_ur_l(ur_pub,temp_q1,0.2,ace,urt)
                        time.sleep(1)
                        x_length=+0.002
                        y_length=+0.002#-0.005
                        depth=0
                        temp_q2=p0.get_inverse_to_box(temp_q1,x_length,y_length,depth)
                        p0.move_ur_l(ur_pub,temp_q2,0.2,ace,urt)
                        time.sleep(1)
                        temp_q3=p0.get_inverse_to_box(temp_q2,0,0,-0.19)
                        p0.move_ur_l(ur_pub,temp_q3,0.2,ace,urt)
                        time.sleep(1)                     
                        os.system("rostopic pub /io_state std_msgs/String '55C8010155' -1")
                        p0.move_ur_l(ur_pub,temp_q2,0.2,ace,urt)
                        time.sleep(1)
                        p0.move_ur_l(ur_pub,cartisian_feedback,0.2,ace,urt)
                        time.sleep(5)
                        rospy.set_param("open_ibvs_flag",0)

                        rospy.set_param("/open_go_to_desire",1)
                        uvlist=[]
                        p0.uv_list_buffer=[]

        # if open_go_to_desire==1 and open_ibvs_flag==0:
        #     print("Desire pick",len(p0.uv_desire_list_buffer))
        #     if len(p0.uv_desire_list_buffer)!=0:
        #         # desire_object=p0.image_space_planning([569,474],[44,65],3,3)
        #         uvlist.append([p0.uv_desire_list_buffer[-1][0],p0.uv_desire_list_buffer[-1][1]])
        #         desiruv.append([333,241])
        #         # desiruv.append([550,339])

        #         feature_error=p0.get_feature_error(desiruv,uvlist[0])
        #         print "Ibvs is ok?---",p0.return_error_ok_desire(feature_error.tolist()[0][0],feature_error.tolist()[0][1])
        #         if p0.return_error_ok_desire(feature_error.tolist()[0][0],feature_error.tolist()[0][1])==False:
        #             print "feature error\n",feature_error
        #             u_error_pub.publish(feature_error.tolist()[0][0])
        #             v_error_pub.publish(feature_error.tolist()[0][1])

        #             q_now=ur_reader.ave_ur_pose

        #             detaangular=p0.get_deta_joint_angular(detat,uvlist, z, desiruv, uvlist[0], q_now)

        #             q_pub_now=p0.get_joint_angular(q_now,detaangular)
        #             print "q_now\n", q_now
        #             print "q_pub_now\n",q_pub_now
        #             down_to_q=q_pub_now
        #             ss = "movej([" + str(q_pub_now[0]) + "," + str(q_pub_now[1]) + "," + str(q_pub_now[2]) + "," + str(
        #                 q_pub_now[3]) + "," + str(q_pub_now[4]) + "," + str(q_pub_now[5]) + "]," + "a=" + str(ace) + "," + "v=" + str(
        #                 vel) + "," + "t=" + str(urt) + ")"
        #             # print ss
        #             ur_pub.publish(ss)
        #         if p0.return_error_ok_desire(feature_error.tolist()[0][0],feature_error.tolist()[0][1])==True:
        #             depth=-0.15
        #             temp_q=p0.get_inverse_to_box(down_to_q,0,0,depth)
        #             p0.move_ur_l(ur_pub,temp_q,0.2,ace,urt)
        #             time.sleep(1)
        #             if place_count>=4:
        #                 pose_place=p0.caculate_place_pose_from_ref(temp_q,0)
        #             else:
        #                 pose_place=p0.caculate_place_pose_from_ref(temp_q,1)
        #             rospy.set_param("/open_go_to_object",0)
        #             p0.move_ur_l(ur_pub,pose_place[0],0.2,ace,urt)
        #             time.sleep(1)
        #             p0.move_ur_l(ur_pub,pose_place[1],0.2,ace,urt)
        #             time.sleep(1)
        #             p0.move_ur_l(ur_pub,pose_place[place_count],0.2,ace,urt)
        #             time.sleep(1)

        #             temp_q_place=p0.get_inverse_to_box(pose_place[place_count],0,0,-0.44)
        #             p0.move_ur_l(ur_pub,temp_q_place,0.2,ace,urt)
        #             time.sleep(1)   
        #             os.system("rostopic pub /io_state std_msgs/String '55C8010055' -1")   
                    
        #             # p0.move_ur_l(ur_pub,cartisian_feedback,0.2,ace,urt)#for debug
        #             p0.move_ur_l(ur_pub,pose_place[place_count],0.2,ace,urt)
        #             time.sleep(2)
                    
        #             p0.uv_desire_list_buffer=[]
        #             rospy.set_param("open_go_desire_flag",0)
        #             p0.move_ur_l(ur_pub,start_angular_back,0.2,ace,urt)
        #             time.sleep(5)
        #             rospy.set_param("/open_go_to_desire",0)

        #             rospy.set_param("/open_go_to_object",1)
        #             rospy.set_param("open_ibvs_flag",1)
        #             place_count+=1


        rate.sleep()


if __name__=="__main__":
    main()