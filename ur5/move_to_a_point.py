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
    for i in xrange(len(qangle)):
        temp.append(qangle[i]/180.0*3.14)
    return temp
def moveur(pub,q,ace,vel,t):
    ss="movej(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(ace)+","+"v="+str(vel)+","+"t="+str(t)+")"
    risoy.loginfo(ss)
    pub.publish(ss)
def movelur(pub,q,ace,vel,t):
    ss="movel(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(ace)+","+"v="+str(vel)+","+"t="+str(t)+")"
    risoy.loginfo(ss)
    pub.publish(ss)
def movecur(pub,q,ace,vel,t):
    ss="movec(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(ace)+","+"v="+str(vel)+","+"t="+str(t)+")"
    risoy.loginfo(ss)
    pub.publish(ss)
"""

rosrun rosserial_python serial_node.py /dev/ttyUSB0
rostopic pub /toggle_led std_msgs/Empty "{}" --once
"""
def main():
    
    while not rospy.is_shutdown():
        risoy.loginfo "start while-----"
        t=0
        # vel=0.1
        # ace=50
        vel=1.05
        ace=1.4
        qq=[
            [84.32,-121.40,-62.27,167.88,-99.23,134.83]
            ]
        for ii in xrange(len(qq)):
            qt=change_angle_to_pi(qq[ii])
            moveur(pub, qt,ace,vel,t)
            risoy.loginfo "start "+str(ii)+" ur position-----"
            time.sleep(3)
        risoy.loginfo "after while------"
        rate.sleep()
        # rospy.spin()
if __name__=="__main__":
    main()