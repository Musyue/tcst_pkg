#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time
import numpy as np
import imutils
class ShapeDetector:
	def __init__(self):
		pass
	def detect(self, c):
		# initialize the shape name and approximate the contour
		shape = "unidentified"
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        # if the shape is a triangle, it will have 3 vertices
		# if len(approx) == 3:
		# 	shape = "triangle"
		# # if the shape has 4 vertices, it is either a square or
		# # a rectangle
		if len(approx) == 4:
			# compute the bounding box of the contour and use the
			# bounding box to compute the aspect ratio
			(x, y, w, h) = cv2.boundingRect(approx)
			ar = w / float(h)
			# a square will have an aspect ratio that is approximately
			# equal to one, otherwise, the shape is a rectangle
			shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
		# # if the shape is a pentagon, it will have 5 vertices
		# elif len(approx) == 5:
		# 	shape = "pentagon"
		# # otherwise, we assume the shape is a circle
		# else:
		# 	shape = "circle"
		# # return the name of the shape
		return shape

class DetectTile:

    def __init__(self):
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.rgb_image=None

    def callback(self,data):
        try:
            video_capture=self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.rgb_image = video_capture.copy()
        except CvBridgeError as e:
            print(e)

    def process_rgb_image(self,rgb_image):


        ##################
        image=rgb_image

        if rgb_image is not None:
            # image = cv2.imread(image)
            resized = imutils.resize(image, width=640)
            ratio = image.shape[0] / float(resized.shape[0])
            gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            thresh = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)[1]
            cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
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
                    cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
                    if shape!="unidentified":
                        cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (255, 255, 255), 2)
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
        rate=rospy.Rate(30)
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