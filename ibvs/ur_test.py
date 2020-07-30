#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import time
from ur5_kinematics import Kinematic
rospy.init_node("move_ur5_by_urscript")
pub=rospy.Publisher("/ur_driver/URScript",String,queue_size=10)
rate=rospy.Rate(30)
def change_angle_to_pi(qangle):
    temp=[]
    for i in range(len(qangle)):
        temp.append(qangle[i]/180.0*3.14)
    return temp
def moveur(pub,q,ace,vel,t):
    ss="movej(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(ace)+","+"v="+str(vel)+","+"t="+str(t)+")"
    rospy.loginfo(ss)
    pub.publish(ss)
def movelur(pub,q,ace,vel,t):
    ss="movel(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(ace)+","+"v="+str(vel)+","+"t="+str(t)+")"
    rospy.loginfo(ss)
    pub.publish(ss)
def movecur(pub,q,ace,vel,t):
    ss="movec(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(ace)+","+"v="+str(vel)+","+"t="+str(t)+")"
    rospy.loginfo(ss)
    pub.publish(ss)
def get_inverse_to_box(pub_now,x_length,y_length,depth):
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
def caculate_place_pose_from_ref(pub_now):
    final={}
    x_length=0.035
    y_length=0.035
    depth=0
    pos0=get_inverse_to_box(pub_now,x_length,y_length,depth)
    rospy.logerr("pos0"+str(pos0))
    final.update({0:pos0})
    x_length=-0.08
    y_length=0.08
    depth=0
    pos1=get_inverse_to_box(pos0,x_length,y_length,depth)
    final.update({1:pos1})
    rospy.logerr("pos1"+str(pos1))
    x_length=0.09
    y_length=0.09
    depth=0
    pos2=get_inverse_to_box(pos1,x_length,y_length,depth)
    final.update({2:pos2})
    x_length=0.09
    y_length=0.09
    depth=0
    pos3=get_inverse_to_box(pos2,x_length,y_length,depth)
    final.update({3:pos3})
    x_length=0.08
    y_length=-0.08
    depth=0
    pos4=get_inverse_to_box(pos1,x_length,y_length,depth)
    final.update({4:pos4}) 
    x_length=0.08
    y_length=-0.08
    depth=0
    pos5=get_inverse_to_box(pos4,x_length,y_length,depth)
    final.update({5:pos5})   

    x_length=0.08
    y_length=-0.08
    depth=0
    pos6=get_inverse_to_box(pos2,x_length,y_length,depth)
    final.update({6:pos6}) 
    x_length=0.08
    y_length=-0.08
    depth=0
    pos7=get_inverse_to_box(pos6,x_length,y_length,depth)
    final.update({7:pos7})
    x_length=0.08
    y_length=-0.08
    depth=0
    pos8=get_inverse_to_box(pos3,x_length,y_length,depth)
    final.update({8:pos8}) 
    x_length=0.08
    y_length=-0.08
    depth=0
    pos9=get_inverse_to_box(pos8,x_length,y_length,depth)
    final.update({9:pos9})
    return final


#cable y negtive
"""
rosrun rosserial_python serial_node.py /dev/ttyUSB0
rostopic pub /toggle_led std_msgs/Empty "{}" --once
"""
def main():
    
    while not rospy.is_shutdown():
        rospy.loginfo("start while-----")
        t=0
        vel=0.1
        ace=50
        # vel=1.05
        # ace=1.4
        qq=[
            # [6.15,-95.53,-80.33,-94.89,92.00,177.78]
            # [55.44,-97.74,-77.52,-95.14,91.78,135.72]
            # [126.12,-75.21,46.26,297.74,-89.49,168.82],#goal
            [44.87,-70.67,40.06,299.46,-89.69,175.71]#start
            # [151.48,-64.77,31.82,301.96,-90.19,194.16]
            # [138.35,-55.49,103.27,220.30,-89.8,181.00]
            ]
        # for ii in range(len(qq)):
            # qt=change_angle_to_pi(qq[ii])
            # # qt=[2.4146897759547734, -1.1350234765509686, 0.5554777536842516, 5.258265530917523, -1.5663199722357237, 3.1563836819776188]
            # moveur(pub, qt,ace,vel,t)
            # # movecur(pub, qt,ace,vel,t)
            # rospy.loginfo("start "+str(ii)+" ur position-----")
            # time.sleep(3)
        qt=[2.4146897759547734, -1.1350234765509686, 0.5554777536842516, 5.258265530917523, -1.5663199722357237, 3.1563836819776188]
        qt=change_angle_to_pi(qq[0])
        qt=[2.4416595885844266, -1.1078617498197416, 0.5115627939858497, 5.276038905750565, -1.5670248793914165, 3.1837172913103657]#goal
        qt=[0.7024339580432882, -1.1993113429190734, 0.6491743365862221, 5.236692470485026, -1.5631476047642547, 2.9838936463128927]
        x_length=0.03
        y_length=0.03
        depth=0
        temp_q=get_inverse_to_box(qt,x_length,y_length,depth)
        movelur(pub, temp_q,ace,vel,t)
        time.sleep(3)        
        # kk=caculate_place_pose_from_ref(temp_q)
        # print(kk)
        # for i in range(len(kk)):
        #     movelur(pub, kk[i],ace,vel,t)
        #     time.sleep(3)
        # movelur(pub, qt,ace,vel,t)
        # time.sleep(3)
        # x_length=0.03
        # y_length=0.03
        # depth=0
        # kk=get_inverse_to_box(qt,x_length,y_length,depth)
        # movelur(pub, kk,ace,vel,t)
        # time.sleep(3)
        # x_length=-0.09
        # y_length=0.09
        # depth=0
        # kk=get_inverse_to_box(kk,x_length,y_length,depth)
        # movelur(pub, kk,ace,vel,t)  
        # time.sleep(3)
        # rospy.loginfo("after while------")
        rate.sleep()
        # rospy.spin()
if __name__=="__main__":
    main()