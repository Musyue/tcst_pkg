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
        self.The_max_box_num=9
        self.ref_point=[]
        self.count_desire=0
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
    def draw_place_box(self,starting_point,rgb_image):
        ref_point_letftop=(starting_point[0],starting_point[1])
        ref_point_rightbottom=(starting_point[2],starting_point[3])
        a=ref_point_rightbottom[0]-ref_point_letftop[0]+10
        b=ref_point_rightbottom[1]-ref_point_letftop[1]+10
        box=[
            [ref_point_letftop,ref_point_rightbottom],
            [[ref_point_letftop[0]+a,ref_point_letftop[1]],[ref_point_rightbottom[0]+a,ref_point_rightbottom[1]]],
            [[ref_point_letftop[0]+2*a,ref_point_letftop[1]],[ref_point_rightbottom[0]+2*a,ref_point_rightbottom[1]]],
            [[ref_point_letftop[0],ref_point_letftop[1]-b],[ref_point_rightbottom[0],ref_point_rightbottom[1]-b]],
            [[ref_point_letftop[0],ref_point_letftop[1]-2*b],[ref_point_rightbottom[0],ref_point_rightbottom[1]-2*b]],

            [[ref_point_letftop[0]+a,ref_point_letftop[1]-b],[ref_point_rightbottom[0]+a,ref_point_rightbottom[1]-b]],
            [[ref_point_letftop[0]+2*a,ref_point_letftop[1]-b],[ref_point_rightbottom[0]+2*a,ref_point_rightbottom[1]-b]],
            [[ref_point_letftop[0]+a,ref_point_letftop[1]-2*b],[ref_point_rightbottom[0]+a,ref_point_rightbottom[1]-2*b]],
            [[ref_point_letftop[0]+2*a,ref_point_letftop[1]-2*b],[ref_point_rightbottom[0]+2*a,ref_point_rightbottom[1]-2*b]],

        ]

        for i in range(self.The_max_box_num):
            ptLeftTop = tuple(box[i][0])
            ptRightBottom = tuple(box[i][1])

            point_color = (0, 255, 0) # BGR
            thickness = 2 
            lineType = 4
            cv2.rectangle(rgb_image, ptLeftTop, ptRightBottom, point_color, thickness, lineType)
    def process_rgb_image(self,rgb_image):
        open_go_to_object=rospy.get_param("open_go_to_object")
        
        open_go_to_desire=rospy.get_param("open_go_to_desire")
        box_count=rospy.get_param("box_count")
        rospy.loginfo("open_go_to_object,open_go_to_desire"+str(open_go_to_object)+","+str(open_go_to_desire))
        pub_data=[]
        if rgb_image is not None:
            try:
                if open_go_to_object==1 or open_go_to_desire==1:
                    if len(self.bounding_box_list)!=0:
                        for i in range(len(self.bounding_box_list)):
                            Cx=(self.bounding_box_list[i][0]+self.bounding_box_list[i][2])/2
                            Cy=(self.bounding_box_list[i][1]+self.bounding_box_list[i][3])/2
                            if open_go_to_desire:
                                area=(self.bounding_box_list[i][2]-self.bounding_box_list[i][0])*(-self.bounding_box_list[i][1]+self.bounding_box_list[i][3])
                                rospy.loginfo("bounding box area "+str(area))
                                if area>8000 and area<20000 and Cx<400:
                                    pub_data.append([Cx,Cy])
                            else:
                                pub_data.append([Cx,Cy])
                            if Cx<=203 and Cx>=158 and Cy<=415 and Cy>=350:
                                self.ref_point=[self.bounding_box_list[i][0],self.bounding_box_list[i][1],self.bounding_box_list[i][2],self.bounding_box_list[i][3]]
                            cv2.circle(rgb_image, (Cx,Cy), 10, (0, 255, 255), -1)
                            cv2.putText(rgb_image, str((Cx,Cy)), (Cx-20,Cy+40), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 2)
                        
                        if open_go_to_object:
                            pub_data = sorted(pub_data,key=lambda x:(x[1],x[0]))
                            print(pub_data)
                            self.object_pub.publish(str(pub_data[-1]))
                            for i in range(len(pub_data)):
                                cv2.putText(rgb_image, str(i), (pub_data[i][0]-40,pub_data[i][1]+20), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 0, 255), 2)
                        if open_go_to_desire:
                            if box_count<=2:
                                pub_data = sorted(pub_data,key=lambda x:(x[1],x[0]))
                                self.desire_pub.publish(str(pub_data[-1]))
                            else:
                                pub_data=sorted(pub_data,key=lambda x:(x[0],x[1]))
                                self.desire_pub.publish(str(pub_data[0]))
                            pub_data=sorted(pub_data,key=lambda x:(x[0],x[1]))
                            pub_data1=sorted(pub_data,key=lambda x:(x[1],x[0]))
                            rospy.logerr("pub_data--->"+str(pub_data))
                            rospy.logerr("pub_data1--->"+str(pub_data1))
                            self.draw_place_box(self.ref_point,rgb_image)
                            for i in range(len(pub_data)):

                                cv2.putText(rgb_image, str(i), (pub_data[len(pub_data)-1-i][0]-40,pub_data[len(pub_data)-1-i][1]+20), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 0, 255), 2)

            except:
                pass

            self.bounding_box_list=[]
            box_count=0

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
        rate=rospy.Rate(10)
        
        while not rospy.is_shutdown():
            k.process_rgb_image(k.rgb_image)
            rate.sleep()

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down cv_bridge_test node.")
        cv2.destroyAllWindows()
if __name__=="__main__":
    main()
