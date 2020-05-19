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
import re,os
class ShapeDetector:
    def __init__(self):
        pass
    def detect(self, c):
        # initialize the shape name and approximate the contour
        peri = cv2.arcLength(c, True)
        area=cv2.contourArea(c,True)
        approx = cv2.approxPolyDP(c, 0.0001 * peri, True)
        print(peri)
        if len(approx) >= 4 and peri<800 and peri>200:
            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            # (x, y, w, h) = cv2.boundingRect(approx)
            # ar = w / float(h)
            # print(x, y, w, h,w*h)
            # if w*h<20000 and w*h >10000:
            #     rospy.loginfo("This is ok--")
            return True
        return False
    def detect_desire(self, c):
        # initialize the shape name and approximate the contour
        peri = cv2.arcLength(c, True)
        area=cv2.contourArea(c,True)
        approx = cv2.approxPolyDP(c, 0.0001 * peri, True)
        print(peri)
        if len(approx) >= 4 and peri>800:
            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            # (x, y, w, h) = cv2.boundingRect(approx)
            # ar = w / float(h)
            # print(x, y, w, h,w*h)
            # if w*h<20000 and w*h >10000:
            #     rospy.loginfo("This is ok--")
            return True
        return False

class DetectTile:
    def __init__(self):
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.object_pub = rospy.Publisher("object_data_pub", String, queue_size=1)
        self.desire_pub = rospy.Publisher("desire_data_pub", String, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.rgb_image=None
        self.object_flag_store_onece=0
    def callback(self,data):
        try:
            video_capture=self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.rgb_image = video_capture.copy()
        except CvBridgeError as e:
            print(e)

    def process_rgb_image(self,rgb_image):


        ##################
        image=rgb_image
        open_go_to_object=rospy.get_param("open_go_to_object")
        print("open_go_to_object",open_go_to_object)
        open_go_to_desire=rospy.get_param("open_go_to_desire")
        if rgb_image is not None:
            # image = cv2.imread(image)
            resized = imutils.resize(image, width=640)
            ratio = image.shape[0] / float(resized.shape[0])
            gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
            if open_go_to_object==1:
                blurred = cv2.GaussianBlur(gray, (11,11), 0)
                thresh = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)[1]
                cnts = cv2.findContours(thresh.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                cnts = imutils.grab_contours(cnts)
                sd = ShapeDetector()
                for c in cnts:
                    # compute the center of the contour, then detect the name of the
                    # shape using only the contour
                    M = cv2.moments(c)
                    if M["m00"]!=0:
                        cX = int((M["m10"] / M["m00"]) * ratio)
                        cY = int((M["m01"] / M["m00"]) * ratio)
                        shape = sd.detect(c)
                        # multiply the contour (x, y)-coordinates by the resize ratio,
                        # then draw the contours and the name of the shape on the image
                        c = c.astype("float")
                        c *= ratio
                        c = c.astype("int")
                        
                        if shape==True:
                            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
                            cv2.putText(image, str((cX, cY)), (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5, (255, 0, 0), 2)
                            print("objtct publish------",str((cX, cY)))
                            self.object_pub.publish(str((cX, cY)))


                # show the output image
                cv2.namedWindow( 'Image', cv2.WINDOW_NORMAL)
                cv2.imshow("Image", image)
                cv2.namedWindow( 'thresh', cv2.WINDOW_NORMAL)
                cv2.imshow( 'thresh', thresh )
                cv2.waitKey(8)

                    # # 再将opencv格式额数据转换成ros image格式的数据发布
                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(rgb_image, "bgr8"))
                except CvBridgeError as e:
                    print(e)
            if open_go_to_desire==1:
                blurred = cv2.GaussianBlur(gray, (11,11), 0)
                thresh = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)[1]
                cnts = cv2.findContours(thresh.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                cnts = imutils.grab_contours(cnts)
                sd = ShapeDetector()
                for c in cnts:
                    # compute the center of the contour, then detect the name of the
                    # shape using only the contour
                    M = cv2.moments(c)
                    if M["m00"]!=0:
                        cX = int((M["m10"] / M["m00"]) * ratio)
                        cY = int((M["m01"] / M["m00"]) * ratio)
                        shape = sd.detect_desire(c)
                        # multiply the contour (x, y)-coordinates by the resize ratio,
                        # then draw the contours and the name of the shape on the image
                        c = c.astype("float")
                        c *= ratio
                        c = c.astype("int")
                        
                        if shape==True:
                            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
                            cv2.putText(image, str((cX, cY)), (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5, (255, 0, 0), 2)
                            print("publish------",str((cX, cY)))
                            self.object_pub.publish(str((cX, cY)))


                        # show the output image
                cv2.namedWindow( 'Image', cv2.WINDOW_NORMAL)
                cv2.imshow("Image", image)
                cv2.namedWindow( 'thresh', cv2.WINDOW_NORMAL)
                cv2.imshow( 'thresh', thresh )
                cv2.waitKey(8)

                # # 再将opencv格式额数据转换成ros image格式的数据发布
                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(rgb_image, "bgr8"))
                except CvBridgeError as e:
                    print(e)


def main():
    try:
        # 初始化ros节点
        rospy.init_node("cv_bridge_test")
        rospy.loginfo("Starting cv_bridge_test node")
        k=DetectTile()
        rate=rospy.Rate(1)
        while not rospy.is_shutdown():
            k.process_rgb_image(k.rgb_image)
            # cen=k.process_rgb_image(k.rgb_image)
            # print "cenpixel\n",cen
            rate.sleep()
            # time.sleep(1)
       # rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node."
        cv2.destroyAllWindows()
if __name__=="__main__":
    main()
