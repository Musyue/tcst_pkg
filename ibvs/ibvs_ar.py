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

from code_flags_sub import *
from std_msgs.msg import UInt8
"""
os.system("rostopic pub io_state std_msgs/String "55C8010155" --once")
"""
class TilingVisionControl():
    def __init__(self,nodename,urdfname,detat,lamda,califilename,camf,kappa=0.7,delta=5):
        self.nodename=nodename
        self.califilename=califilename
        self.urdfname=urdfname
        self.camf=camf
        self.detat=detat
        self.kappa=kappa
        self.delta=delta
        self.lamda=lamda


        self.ledstate=None
        self.changeuv=None

    def Init_node(self):
        rospy.init_node(self.nodename)

        ur_pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)

    def Get_ur_X(self,info):

        aa = Get_X_from_cali_quaternion(info)
        aa = numpy.mat(aa)
        # print "X", aa
        return aa.reshape((4, 4))
    def q2t(self,pose_ave):
        for i in xrange(len(pose_ave)):
            trans=pose_ave[i][:3]
            mtrans=numpy.matrix(trans)
            print "mtrans",mtrans
            addnum=[0,0,0,1]
            addnum=numpy.matrix(addnum)
            # print "addnum",addnum
            s = pose_ave[i][6]
            v1 = pose_ave[i][3]
            v2 = pose_ave[i][4]
            v3 = pose_ave[i][5]
            q = quaternion(s, v1, v2, v3)
            # print "unit",q.unit()
            q2ro=q.r().T

            temp=numpy.vstack((numpy.hstack((q2ro, mtrans.T)), addnum))
            # print "q2r-------\n",i,numpy.vstack((numpy.hstack((q2ro,mtrans.T)),addnum))
            return temp
    def get_jacabian_from_joint(self,urdfname,jointq):
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
        # # print pose
        # #print list(pose)
        # q0=Kinematic(q)
        # if flag==1:
        #     q_ik=q0.best_sol_for_other_py( [1.] * 6, 0, q0.Forward())
        # else:
        #     q_ik = kdl_kin.inverse(pose)  # inverse kinematics
        # # print "----------iverse-------------------\n", q_ik
        #
        # if q_ik is not None:
        #     pose_sol = kdl_kin.forward(q_ik)  # should equal pose
        #     print "------------------forward ------------------\n",pose_sol

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
    def vis2jac(self,uv,z):
        cam=self.get_cam_data()
        rh0=[0.0000032,0.0000032]
        camf=self.camf#m
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
        return kk.reshape((1,2))
    """
    #cam speed (udot,vdot)(xdot,ydot,zdot,wxdot,wydot,wzdot)
    #get camera frame speed,you must change to ee frame
    #uvm means now uv
    """

    def get_cam_vdot(self,uvm,z,desireuv,nowuv):
        J=self.vis2jac_mt1(uvm,z)
        JJ=numpy.linalg.pinv(J)
        e=self.get_feature_error(desireuv,nowuv)
        vdot=self.lamda*numpy.dot(JJ,e.T)
        return vdot

    #samebody tranlasition to jacbian
    #joint speed (q0dot,q1dot,q2dot,q3dot,q4dot,q5dot)
    def get_joint_speed(self,uvm,z,desireuv,nowuv,q,info):
        #1,get base to ee jacabian
        Jacabian_joint,T_06=self.get_jacabian_from_joint(self.urdfname,q)
        #2,get ee(AX=XB) to camera frame jacabian
        X=self.Get_ur_X(info)#numpu array
        ebT=T_06
        #tr2jac
        jac = tr2jac(X,1)
        jac_b2e=tr2jac(T_06,0)
        #print "------X",X
        inv_X_jac = jac.I
        #get ee speed
        #print "tr2jac-----\n",jac
        cam_speed = self.get_cam_vdot(uvm, z, desireuv, nowuv)
        # print "cam_speed--------",cam_speed
        ee_speed_in_eeframe = numpy.dot(inv_X_jac, cam_speed)
        v_list = ee_speed_in_eeframe.reshape((1, 6)).tolist()[0]
        #[z,y,]
        flag_list = [0, 1, 1, 0, 0, 0]
        vdot_z = [1.0 * v_list[i] * flag_list[i] for i in range(6)]
        ee_speed_in_base = numpy.dot(jac_b2e.I, numpy.mat(vdot_z).T)
        # print "ee_speed-----before changing--------",ee_speed_in_base

        # print("ee_speed_after--------------\n",vdot_z)
        j_speed=numpy.dot(Jacabian_joint.I,ee_speed_in_base)
        return j_speed
    def get_deta_joint_angular(self,j_speed):
        #print j_speed
        joint_angular=float(self.detat)*numpy.array(j_speed)
        #print '-------joint_angular-----\n',joint_angular
        return joint_angular

    def get_joint_angular(self,qnow,detajoint):
        #result=[]
        listangular=[]
        for i in range(len(detajoint.tolist())):
            listangular.append(detajoint.tolist()[i][0]+qnow[i])
        # print "list",detajoint.tolist()
        return listangular
    def get_ar_frame_in_base(self,q_now,ar_pose,info):
        X=self.Get_ur_X(info)
        print "x",X
        k=Kinematic()
        Tbe=k.Forward(q_now)
        print "Tbe",numpy.array(Tbe).reshape((4,4))
        # print "ar_pose[0]",ar_pose
        Tea=self.q2t(ar_pose)
        TeaxX=numpy.dot(Tea,X)
        TeaxXxTbe=numpy.dot(TeaxX,numpy.array(Tbe).reshape((4,4)))

        print "TeaxX",TeaxX
        print "Tea",Tea
        print "TeaxXxTbe",TeaxXxTbe
    def get_ar_frame_in_base1(self,q_now,ar_pose,info):
        X=self.Get_ur_X(info)
        print "x",X
        k=Kinematic()
        Tbe=k.Forward(q_now)
        print "Tbe",numpy.array(Tbe).reshape((4,4))
        # print "ar_pose[0]",ar_pose
        Tea=self.q2t(ar_pose)
        TeaxX=numpy.dot(X,Tea)
        TeaxXxTbe=numpy.dot(numpy.array(Tbe).reshape((4,4)),TeaxX)

        print "TeaxX",TeaxX
        print "Tea",Tea
        print "TeaxXxTbe",TeaxXxTbe
        # TT=self.numpy_array_to_list(TeaxXxTbe)
        # path_road=self.caculating_path_to_pose(2,TT)
        # print "path_road",path_road
        return TeaxXxTbe
    """
    2x2,level=2,3x3,level=3
    """
    def caculating_path(self,level,q_now,ar_pose,info):
        result=[]
        TbeXTca=self.get_ar_frame_in_base1(q_now,ar_pose,info)

        X = self.Get_ur_X(info)
        Tea=self.q2t(ar_pose)
        TeaxX=numpy.dot(X,Tea)
        TT=self.numpy_array_to_list(TbeXTca)
        path_road=self.caculating_path_to_pose(level,TT)
        # print "path_road",path_road
        # path_road=[]
        # T0=[]
        # for i in xrange(len(TT)):
        #     if i ==3:
        #         T0.append(0.47)
        #     else:
        #         T0.append(TT[i])
        # path_road.append(T0)
        print "path_road",path_road
        result_all=[]
        for i in xrange(len(path_road)):
            new_T=numpy.dot(numpy.array(path_road[i]).reshape((4,4)),TeaxX.I)
            print "New T",new_T
            result_all.append(self.numpy_array_to_list(new_T))
        return result_all
    def caculate_pub_joint_for_ur_use_path(self,level,q_now,ar_pose,info):
        weights = [1.] * 6
        result=self.caculating_path(level,q_now,ar_pose,info)
        F=Kinematic()
        result_all_q=[]
        for i in xrange(len(result)):
            k=F.best_sol(weights,q_now,result[i])
            result_all_q.append(k.tolist())
        return result_all_q
    def caculate_feature_pos(self,q_now,ar_pose,info):
        TbeXTca = self.get_ar_frame_in_base1(q_now, ar_pose, info)
        TT = self.numpy_array_to_list(TbeXTca)
        T0=[]
        for i in xrange(len(TT)):
            if i ==3:
                T0.append(0.47)
            else:
                T0.append(TT[i])
        weights = [1.] * 6
        F=Kinematic()
        result_all_q=[]
        k=F.best_sol(weights,q_now,T0)
        result_all_q.append(k.tolist())
        print result_all_q
        return result_all_q
    """
    T.translation is the marker coordinate in base frame,T.Rotaion is the posture in base frame
    level is the layer of tiles,such as 2x2,level=2,3x3,level=3
    """
    def caculating_path_to_pose(self,level,T):
        trans0=[T[3],T[7],T[11]]
        x0=T[3]
        y0=T[7]
        z0=T[11]
        result=[]
        for i in xrange(level):
            if i % 2==0 and i != 0:#even number
                for ii in xrange(level):
                    result.append([x0,y0-0.05-0.1*ii,z0+0.025+0.13+0.05+0.2*(i-1)])#0.075 sucker to camera center
            elif i%2 !=0:#odd number
                for ii in range(level)[::-1]:
                    result.append([x0,y0-0.05-0.1*ii,z0+0.025+0.13+0.05+0.1*i])
            else:
                for ii in xrange(level):
                    result.append([x0,y0-0.05-0.1*ii,z0+0.025+0.13+0.05])
        # print result
        all_result=[]

        for i in xrange(len(result)):
            temp = []
            print "level"+str(i)+"layer"+str(result[i])
            for j in xrange(len(T)):

                if j==3:
                    temp.append(result[i][0])
                elif j==7:
                    temp.append(result[i][1])
                elif j==11:
                    temp.append(result[i][2])
                else:
                    temp.append(T[j])
            all_result.append(temp)

        return all_result
    def numpy_array_to_list(self,T):
        temp=[]
        Tlist=T.tolist()
        for i in xrange(len(Tlist)):
            for ii in Tlist[i]:
                temp.append(ii)
        # print temp
        return temp
    def check_vison_error_is_zero(self,desireuv,nowuv,error):
        feature_error=self.get_feature_error(desireuv,nowuv).reshape((1,2))
        print "feature_error",feature_error.tolist()
        if abs(feature_error.tolist()[0][0])<=error and abs(feature_error.tolist()[0][1])<=error:
            return 1
        else:
            print "Need more time to go to object----------"
            return 0
    def ibvs_run_ur5(self,uvm,z,q,info,xp):

        """
        First,Get the uv to drive ur5
        x=uvm[0]=nowuv

        """
        # xp=self.tile_0_buf[-1][1]
        """
        Second,caculating cam vodt and deta joint speed
        desireuv=xp
        """

        joint_speed_dot=self.get_joint_speed(uvm, z, xp, uvm[0], q, info)
        # print "joint_speed_dot",joint_speed_dot
        """
        Third,caculating deta joint speed
        """
        deta_joint_angular=self.get_deta_joint_angular(joint_speed_dot)
        # print "deta_joint_angular",deta_joint_angular
        """
        Fourth,get joint angular
        """
        pub_joint=self.get_joint_angular(q, deta_joint_angular)
        # print "pub_joint",pub_joint
        return pub_joint

    def Move_ur(self,q_pub_now,ace,vel,urt):
        ss = "movej([" + str(q_pub_now[0]) + "," + str(q_pub_now[1]) + "," + str(q_pub_now[2]) + "," + str(
            q_pub_now[3]) + "," + str(q_pub_now[4]) + "," + str(q_pub_now[5]) + "]," + "a=" + str(
            ace) + "," + "v=" + str(
            vel) + "," + "t=" + str(urt) + ")"
        print "ss",ss
        return ss
    def Open_sucking_close_IoBoard(self,flag):
        Protocol="55C8010"+str(flag)+"55"
        Pub_str='rostopic pub io_state std_msgs/String '+Protocol+' --once'
        os.system(Pub_str)


    """
    Low power is valid
    Before using this function,you must make sure,P13 is high.Led open
    """
    def Open_sucking_close_Ardunio(self):
        Protocol="{}"
        Pub_str='rostopic pub /toggle_led std_msgs/Empty '+Protocol+' --once'
        os.system(Pub_str)
    def move_sucker_to_tile(self,z_distance,tile_width,q_now):
        Kine=Kinematic()
        T = Kine.Forward(q_now)
        # moveur(pub, qt,ace,vel,t)
        print "Now T",numpy.array(T).reshape(4,4)
        #z_distance=0.33
        new_T = insert_new_xy(T, T[3], T[7], z_distance)  # 解算出来的是基于world的
        q_new = get_IK_from_T(Kine, new_T, q_now).tolist()
        return q_new
    def move_ur_to_desire(self,x_distance,q_now):
        Kine=Kinematic()
        T = Kine.Forward(q_now)
        # moveur(pub, qt,ace,vel,t)
        print "Now T",numpy.array(T).reshape(4,4)
        #z_distance=0.33
        new_T = insert_new_xy(T, x_distance, T[7], T[11])  # 解算出来的是基于world的
        q_new = get_IK_from_T(Kine, new_T, q_now).tolist()
        return q_new
    """
    这是一个非常大的坑，垂直的机械臂驱动，竟然需要改x,y两个坐标才行
    """
    def move_ur_to_desire_vertical(self,x_distance,y_distance,q_now):
        Kine=Kinematic()
        T = Kine.Forward(q_now)
        # moveur(pub, qt,ace,vel,t)
        print "Now T",numpy.array(T).reshape(4,4)
        #z_distance=0.33
        new_T = insert_new_xy(T, x_distance, y_distance, T[11])  # 解算出来的是基于world的
        q_new = get_IK_from_T(Kine, new_T, q_now).tolist()
        return q_new
    def change_angle_to_pi(self,qangle):
        temp = []
        for i in xrange(len(qangle)):
            temp.append(qangle[i] / 180.0 * 3.14)
        return temp
    """
    uv0,初始状态下的uv,level,螺旋贴砖的层次
    """
    def Caculate_desire_uv_for_place(self,level):
        size=100
        uv0=self.tile_0_buf[-1][1]
        id1=uv0
        id2=[uv0[0],uv0[1]-size]
        id3=[uv0[0]-size,uv0[1]-size]
        id4=[uv0[0]-size+39,uv0[1]-18]
        id5=[uv0[0]-size+30,uv0[1]+size-36]
        # return [id1,id2,id3,id4,id5]
        return [id1, id4, id5,id2, id3]
    """
    基于joint q做旋转,末端做90度旋转,rotation angular
    """
    def Rotaion_tool_90(self,q_now,angular):
        q_new=[]
        for i in xrange(len(q_now)):
            if i ==5:
                q_new.append(q_now[i]+(angular*math.pi/180))
            else:
                q_new.append(q_now[i])
        return q_new
    """ read data from yaml, here it temporary uses the list exist"""
    def get_instrinc_param(self):
        data = numpy.array(
            [627.260603, 0.000000, 316.404078, 0.000000, 622.895967, 251.341039, 0.000000, 0.000000, 1.000000])
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
def main():
    #uvlist=[123.0,112.0]
    uvlist=[]
    camf=624.0429 * 1e-03
    #316,251
    camera_center=[316,251]
    # uvcentral_place=[316,251]
    uvcentral_place=[323,174]#320,201#180
    uvcentral = [339,120]#sucking central109-125OK,370,201#368,95#120
    First_joint_angular=[]
    calibinfo=[
        0.109982645426,
        0.114476746567,
        -0.0415924235801,
        0.249492772999,
        0.523487628443,
        0.239612281752,
        0.778652691205
    ]
    urdfname = "/data/ros/ur_ws_yue/src/tilling_robot/urdf/ur5.urdf"
    cailename = "/data/ros/ur_ws_yue/src/tilling_robot/yaml/cam_500_logitech.yaml"
    nodename="ibvs_vision_control"
    tile_width = 0.01 #m

    ace=1.4
    vel=1.05
    urt=0
    detat=0.05
    ratet=15
    z_distance=0
    lamda=3.666666
    # lamda = 7.066666
    level=2
    z=0.45
    ur_reader = Urposition()
    ur_sub = rospy.Subscriber("/joint_states", JointState, ur_reader.callback)
    u_error_pub = rospy.Publisher("/object_feature_u_error", Float64, queue_size=10)
    v_error_pub = rospy.Publisher("/object_feature_v_error", Float64, queue_size=10)


    ar_reader = arReader()
    ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ar_reader.callback)

    """
    Those two flag use to make sucker sucking tile
    """
    object_flag=0
    open_vison_flag= 0
    desire_flag=0
    turn_on_vision_flag=0
    """
    nodename,urdfname,delta,kappa,lamda,califilename,camf)
    """
    F0=TilingVisionControl(nodename,urdfname,detat,lamda,cailename,camf)
    ur_pub, sucker_center_obj_pub, sucker_center_desire_pub=F0.Init_node()
    rate = rospy.Rate(ratet)
    First_joint_angular = ur_reader.ave_ur_pose

    Object_joint_angular_vision_state=[]
    Object_joint_angular_sucking_state = [-58.85,-90.39,-88.02,-91.27,90.11,69.44]#66.02
    Object_joint_angular_sucking_state_new= F0.change_angle_to_pi(Object_joint_angular_sucking_state)
    Desire_joint_angular_vision_state=[]
    Desire_joint_angular_place_state = [-175.78,-86.57,-120.76,29.08,86.95,43.05]
    Desire_joint_angular_place_state_new = F0.change_angle_to_pi(Desire_joint_angular_place_state)
    print "First_joint_angular",First_joint_angular

    # tile_2=[-217.59,-105.73,-89.85,22.94,88.92,45.29]
    # tile_2_new=F0.change_angle_to_pi(tile_2)
    # tile_3=[-217.92,-104.96,-82.53,16.9,88.92,45.27]
    # tile_3_new=F0.change_angle_to_pi(tile_3)
    open_to_desire_pose_flag = 0#用于打开关闭ur移动到贴砖位置
    count_for_tile=0
    open_suking_flag=0
    open_roation_flag=0
    open_move_to_desire_flag=0
    open_vison_flag_desire=0
    open_suking_flag_desire=0
    open_move_to_object_flag=0
    tile_nums=0#为了记录吸第几块砖
    msg_uv_o=uv()
    msg_uv_d = uv()
    pos_dict_temp=[]
    open_store_ar_pose=0
    count_time_line=0
    while not rospy.is_shutdown():
        desire_tile_path_cacu=[]
        uvlist=[]
        try:

            q_now = ur_reader.ave_ur_pose
            print "q_now",q_now
            if len(q_now) != 0:
                if open_vison_flag_desire == 0:
                    pos_dict = ar_reader.ave_pos_dict
                    uvmm = [camera_center]
                    print "目标空间第一步，打开目标空间的视觉伺服程序"
                    # xpp=F0.tile_0_buf[-1][1]
                    uvlist.append(F0.get_uv_from_ar(pos_dict[0][:3])[:2])
                    # print uvlist
                    uv0=uvlist[0]

                    q_pub_now_d=F0.ibvs_run_ur5(uvmm,z,q_now,calibinfo,uv0)
                    MoveUrString_1=F0.Move_ur(q_pub_now_d, ace, vel, urt)
                    ur_pub.publish(MoveUrString_1)
                    feature_error_zero_flag_d = F0.check_vison_error_is_zero(uv0,
                                                                             camera_center, 3)
                    """
                    if error is zero ,we close vision servo
                    """
                    if feature_error_zero_flag_d:
                        print """
                        目标空间第二步，计算贴砖路径
                        """
                        # open_suking_flag_desire = 1
                        open_vison_flag_desire =1
                        # tile_nums+=1
                        time.sleep(2.0)
                        print "q_now before all_pub_q"
                        all_pub_q = F0.caculate_pub_joint_for_ur_use_path(level, q_now, pos_dict, calibinfo)
                        # print "---------you are now in desire space-----------"

                """
                第四步，关闭视觉伺服，打开轨迹规划，并释放瓷砖
                """
                print "open_vison_flag_desire ==1 and open_suking_flag_desire==1",open_vison_flag_desire,open_suking_flag_desire
                if open_vison_flag_desire ==1 :
                    # MoveUrString=F0.Move_ur(all_pub_q[0], ace, vel, urt)
                    # ur_pub.publish(MoveUrString)
                    # time.sleep(5)
                    """
                    First,go to place sucking tile position
                    """
                    print "第四步，关闭视觉伺服，打开轨迹规划，并释放瓷砖"
                    count_time_line+=1
                    if open_to_desire_pose_flag==0:
                        MoveUrString=F0.Move_ur(all_pub_q[tile_nums], ace, 0.2, urt)
                        ur_pub.publish(MoveUrString)
                        time.sleep(4)
                        open_to_desire_pose_flag=1
                    x_distance = 0.60
                    if open_to_desire_pose_flag==1 and count_time_line==2:
                        q_before_sucking_d=all_pub_q[tile_nums]
                        q_new_pub_d = F0.move_ur_to_desire(x_distance, all_pub_q[tile_nums])
                        MoveUrString = F0.Move_ur(q_new_pub_d, 0.2, vel, urt)
                        # ur_pub.publish(MoveUrString)
                        # time.sleep(5)/
                        open_suking_flag_desire=1
                        count_time_line=0
                        tile_nums += 1
                    """
                    Second,close air pump for sucking
                    """
                    if open_suking_flag_desire == 1 :
                        F0.Open_sucking_close_Ardunio()
                        open_suking_flag_desire = 0
                        """
                        Third,go back the same point
                        """
                        MoveUrString = F0.Move_ur(q_before_sucking_d, ace, 0.2, urt)
                        # ur_pub.publish(MoveUrString)
                        # time.sleep(1)
                        open_move_to_object_flag = 1
                    if open_move_to_object_flag == 1:
                        """
                        Fourth,move ur to desire position.
                        """
                        print "#########Desire,--Fourth,move ur to object position-------"

                        MoveUrString_2 = F0.Move_ur(Object_joint_angular_sucking_state_new, ace, 0.3, urt)
                        # ur_pub.publish(MoveUrString_2)
                        open_vison_flag = 0
                        open_suking_flag = 0
                        open_move_to_desire_flag = 0
                        open_vison_flag_desire = 0
                        open_move_to_object_flag = 0
                        open_suking_flag_desire = 0
                        time.sleep(7)


            if tile_nums==4:
                tile_nums=0
            rate.sleep()
        except KeyboardInterrupt:

            pass


if __name__=="__main__":
    main()

