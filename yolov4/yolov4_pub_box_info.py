#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time
import numpy as np
import imutils
from std_msgs.msg import String
from darknet_ros_msgs.msg import BoundingBoxes
import re,os

class PUBBoxInfo:
    def __init__(self):

        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.object_pub = rospy.Publisher("object_data_pub", String, queue_size=1)
        self.desire_pub = rospy.Publisher("desire_data_pub", String, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.bounding_box_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bounding_box_callback)
        self.rgb_image=None
        self.bounding_box_list=[]
    def callback(self,data):
        try:
            video_capture=self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.rgb_image = video_capture.copy()
        except CvBridgeError as e:
            print(e)
    def bounding_box_callback(self,msg):
        try:

            # print("0",msg.bounding_boxes[0].xmin)
            # print("1",msg.bounding_boxes[1].xmin,len(msg.bounding_boxes))
            if len(msg.bounding_boxes)!=0:
                for i in range(len(msg.bounding_boxes)):
                    if(len( self.bounding_box_list))>len(msg.bounding_boxes)-1:
                        self.bounding_box_list=self.bounding_box_list[1:]
                        self.bounding_box_list.append([msg.bounding_boxes[i].xmin,msg.bounding_boxes[i].ymin,msg.bounding_boxes[i].xmax,msg.bounding_boxes[i].ymax])
                    else:
                        self.bounding_box_list.append([msg.bounding_boxes[i].xmin,msg.bounding_boxes[i].ymin,msg.bounding_boxes[i].xmax,msg.bounding_boxes[i].ymax])
        except:
            rospy.loginfo("There are no box in camera FOV")
    def process_rgb_image(self,rgb_image):
        open_go_to_object=rospy.get_param("open_go_to_object")
        print("open_go_to_object",open_go_to_object)
        open_go_to_desire=rospy.get_param("open_go_to_desire")
        pub_data=[]
        if rgb_image is not None:

            if open_go_to_object==0 or open_go_to_desire==0:
                if len(self.bounding_box_list)!=0:
                    for i in range(len(self.bounding_box_list)):
                        Cx=(self.bounding_box_list[i][0]+self.bounding_box_list[i][2])/2
                        Cy=(self.bounding_box_list[i][1]+self.bounding_box_list[i][3])/2
                        pub_data.append([Cx,Cy])
                        cv2.circle(rgb_image, (Cx,Cy), 10, (0, 255, 255), -1)
                        cv2.putText(rgb_image, str((Cx,Cy)), (Cx-20,Cy+40), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 2)
                    pub_data = sorted(pub_data,key=lambda x:(x[1],x[0]))
                    print(pub_data)
                    for i in range(len(pub_data)):
                        cv2.putText(rgb_image, str(i), (pub_data[i][0]-40,pub_data[i][1]+20), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 0, 255), 2)
                        


            self.bounding_box_list[]

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(rgb_image, "bgr8"))
            except CvBridgeError as e:
                print(e)


def main():
    try:
        # 初始化ros节点
        rospy.init_node("yolov4_pub_box_info_node")
        rospy.loginfo("Starting yolov4_pub_box_info_node node")
        k=PUBBoxInfo()
        rate=rospy.Rate(1)
        
        while not rospy.is_shutdown():
            k.process_rgb_image(k.rgb_image)
            rate.sleep()

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down cv_bridge_test node.")
        cv2.destroyAllWindows()
if __name__=="__main__":
    main()
