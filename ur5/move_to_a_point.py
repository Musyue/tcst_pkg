#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import time
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
"""

rosrun rosserial_python serial_node.py /dev/ttyUSB0
rostopic pub /toggle_led std_msgs/Empty "{}" --once
"""
def main():
    
    while not rospy.is_shutdown():
        rospy.loginfo("start while-----")
        t=0
        vel=0.3
        ace=50
        # vel=1.05
        # ace=1.4
        qq=[
            # [6.15,-95.53,-80.33,-94.89,92.00,177.78]
            # [55.44,-97.74,-77.52,-95.14,91.78,135.72]
            # [131.12,-87.23,61.08,294.70,-89.59,173.75],#goal,

            [44.87,-70.67,40.06,299.46,-89.69,182.71]#start

            # [0.0, -162.47, 38.60, 74.39, 0.0, 180.0]
            # [0,0,0,0,0,0]
            ]
        for ii in range(len(qq)):
            qt=change_angle_to_pi(qq[ii])
            # qt=[0.7024339580432882, -1.1993113429190734, 0.6491743365862221, 5.236692470485026, -1.5631476047642547, 2.983893646312892]
            moveur(pub,qt,ace,vel,t)

            rospy.loginfo("start "+str(ii)+" ur position-----")
            time.sleep(5)
        rospy.loginfo("after while------")
        rate.sleep()
        # rospy.spin()
if __name__=="__main__":
    main()