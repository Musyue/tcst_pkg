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
                # print dlines
                if len(dlines[0])!=0:
                    for dline in dlines[0]:  
                        x0 = int(round(dline[0][0]))     
                        y0 = int(round(dline[0][1]))    
                        x1 = int(round(dline[0][2]))     
                        y1 = int(round(dline[0][3]))  
                        # print("x0,y0,x1,y1",x0,y0,x1,y1) 
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
        self.avg_data_x=[]
        self.avg_data_y=[]
        self.cross_point_data=[]
        self.pub_cross_data=[]
        self.last_time_desire_pub_data=[] #use for filter
        self.last_time_object_pub_data=[]
        self.obejct_pub_list_temp=[]
    def callback(self,data):
        try:
            video_capture=self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.rgb_image = video_capture.copy()
        except CvBridgeError as e:
            print(e)
    def cross_point(self,line1,line2):#计算交点函数
        x1=line1[0]#取四点坐标
        y1=line1[1]
        x2=line1[2]
        y2=line1[3]
        
        x3=line2[0]
        y3=line2[1]
        x4=line2[2]
        y4=line2[3]
        x=0
        y=0
        if x2-x1!=0:
            k1=(y2-y1)*1.0/(x2-x1)#计算k1,由于点均为整数，需要进行浮点数转化
            b1=y1*1.0-x1*k1*1.0#整型转浮点型是关键
            y=k1*x*1.0+b1*1.0
            if (x4-x3)==0:#L2直线斜率不存在操作
                k2=None
                b2=0
            else:
                k2=(y4-y3)*1.0/(x4-x3)#斜率存在操作
                b2=y3*1.0-x3*k2*1.0

                if k2==None:
                    x=x3
                else:
                    if abs(k1-k2)>0:
                        x=(b2-b1)*1.0/(k1-k2)
                if x!=0 or y!=0:
                    return (int(x),int(y))
        return (0,0)
    def convert_hls(self,image):
        return cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    def select_yellow(self,image,data_min):
        converted = self.convert_hls(image)

        # converted = self.convert_hsv(image)
        lower = np.uint8([0, data_min, 0])
        upper = np.uint8([255,255,255])
        white_mask = cv2.inRange(converted, lower, upper)
        #yellow color mask
        lower = np.uint8([0, data_min, 0])
        upper = np.uint8([255,255,255])
        yellow_mask = cv2.inRange(converted, lower, upper)
        # combine the mask
        mask = cv2.bitwise_or(white_mask, yellow_mask)
        return cv2.bitwise_and(image, image, mask=mask)
    def takeSecond(self,elem):
        return elem[1]
    def takeFirst(self,elem):
        return elem[0]
    def process_rgb_image(self,rgb_image):


        ##################
        image=rgb_image
        cross_point=[]
        self.cross_point_data=[]
        self.avg_data_x=[]
        self.avg_data_y=[]
        pub_data=[]
        open_go_to_object=rospy.get_param("open_go_to_object")
        choose_next_point=rospy.get_param("choose_next_point")
        print("open_go_to_object",open_go_to_object)
        open_go_to_desire=rospy.get_param("open_go_to_desire")
        threshold_min=rospy.get_param("threshold_min")
        data_min=rospy.get_param("color_min")
        if rgb_image is not None:
            # image = cv2.imread(image)
            resized = imutils.resize(image, width=640)
            ratio = image.shape[0] / float(resized.shape[0])
            gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
            if open_go_to_object==1:
                try:
                    cv2.destroyWindow("YHLS_Black_HLS_Space")
                    cv2.destroyWindow("dilated_Black_HLS_Space")
                    cv2.destroyWindow("tile_pixel_frame")
                except:
                    pass
                kernel1 = cv2.getStructuringElement(cv2.MORPH_RECT,(5, 5))
                eroded1 = cv2.erode(gray,kernel1)
                dilated1 = cv2.dilate(eroded1,kernel1)
                dilated1 = cv2.dilate(dilated1,kernel1)
                eroded1 = cv2.erode(dilated1,kernel1)
                blurred = cv2.GaussianBlur(gray, (11,11), 0)
                thresh = cv2.threshold(blurred, threshold_min, 255, cv2.THRESH_BINARY)[1]
                cnts = cv2.findContours(thresh.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                cnts = imutils.grab_contours(cnts)
                sd = ShapeDetector()
                for c in cnts:
                    # compute the center of the contour, select_yellowthen detect the name of the
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
                                    1, (0, 0, 255), 2)
                            # print("objtct publish------",str((cX, cY)))
                            if len(self.obejct_pub_list_temp)>5:
                                self.obejct_pub_list_temp=self.obejct_pub_list_temp[1:]
                                self.obejct_pub_list_temp.append((cX, cY))
                            else:
                                self.obejct_pub_list_temp.append((cX, cY))
                if len(self.obejct_pub_list_temp)!=0:
                    pub_data = sorted(self.obejct_pub_list_temp,key=lambda x:(x[1],x[0]))

                    # self.object_pub.publish(str((cX, cY)))
                    if len(self.last_time_object_pub_data)!=0:
                        if abs(self.last_time_object_pub_data[0]-cX)>200: #滤波
                            self.object_pub.publish(str(self.last_time_object_pub_data))
                            cv2.circle(image, self.last_time_object_pub_data, 10, (0, 255, 255), -1)
                        else:
                            cv2.circle(image, pub_data[0], 10, (0, 255, 255), -1)
                            print("object pub_data-======>",pub_data)
                            # if dd[1]<100 and dd[1]>40:
                            # rospy.loginfo("publisher----%s",str(dd))
                            self.object_pub.publish(str(pub_data[0]))
                            self.last_time_object_pub_data=pub_data[0]
                    else:
                        self.last_time_object_pub_data=pub_data[0]


                # show the output image
                cv2.namedWindow( 'Image', cv2.WINDOW_NORMAL)
                cv2.imshow("Image", image)
                cv2.namedWindow( 'obejct_thresh', cv2.WINDOW_NORMAL)
                cv2.imshow( 'obejct_thresh', thresh )
                cv2.waitKey(8)

                    # # 再将opencv格式额数据转换成ros image格式的数据发布
                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(rgb_image, "bgr8"))
                except CvBridgeError as e:
                    print(e)
            if open_go_to_desire==1:
                try:
                    cv2.destroyWindow("Image")
                    cv2.destroyWindow("obejct_thresh")
                    # pub_data=[]
                except:
                    pass
                YHLS=self.select_yellow(image,data_min)
                gray = cv2.cvtColor(YHLS, cv2.COLOR_BGR2GRAY)
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5, 5))
                eroded = cv2.erode(gray,kernel)
                dilated = cv2.dilate(eroded,kernel)
                dilated = cv2.dilate(dilated,kernel)
                eroded = cv2.erode(dilated,kernel)

                lsd = cv2.createLineSegmentDetector(0, _scale=0.1) 
                dlines = lsd.detect(dilated) 
                # print dlines
                if len(dlines[0])!=0:
                    for dline in dlines[0]:  
                        x0 = int(round(dline[0][0]))     
                        y0 = int(round(dline[0][1]))    
                        x1 = int(round(dline[0][2]))     
                        y1 = int(round(dline[0][3]))  
                        # print("x0,y0,x1,y1",x0,y0,x1,y1) 
                        cross_point.append([x0,y0,x1,y1])  
                        # for dline_1 in dlines[0]:
                        #     x2 = int(round(dline[0][0]))     
                        #     y2 = int(round(dline[0][1]))    
                        #     x3 = int(round(dline[0][2]))     
                        #     y3 = int(round(dline[0][3])) 
                        #     print(x2,y2,x3,y3)
                        #     dd=self.cross_point([x0,y0,x1,y1],[x2,y2,x3,y3])
                        
                        # cv2.circle(image, dd, 10, (0, 255, 0), -1)
                    
                        cv2.line(image, (x0, y0), (x1,y1), (0,0,255), 2, cv2.LINE_AA)
                    for i in range(len(cross_point)):
                        for j in range(len(cross_point)):
                            if i!=j:
                                dd=self.cross_point(cross_point[i],cross_point[j])
                                if dd[0]>0  and dd[0] <545 and dd[1]>0 and dd[1]<450:
                                    # if dd[0]<545 and dd[1]<450:
                                    self.avg_data_x.append(dd[0])
                                    self.avg_data_y.append(dd[1])
                                    print("dd",dd)
                                    cv2.circle(image, dd, 10, (0, 0, 255), -1)
                                    if len(self.cross_point_data)>5:
                                        self.cross_point_data=self.cross_point_data[1:]
                                        self.cross_point_data.append(dd)
                                    else:
                                        self.cross_point_data.append(dd)

                    # self.cross_point_data.sort(key=self.takeFirst)

                    if len(self.cross_point_data)!=0:
                        pub_data = sorted(self.cross_point_data,key=lambda x:(x[1],x[0]))
                        print("desire pub_data-======>",pub_data)
                        
                        if len(self.last_time_desire_pub_data)!=0:
                            if abs(self.last_time_desire_pub_data[0]-pub_data[choose_next_point][0])>200:
                                self.desire_pub.publish(str(self.last_time_desire_pub_data))
                                cv2.circle(image, self.last_time_desire_pub_data, 10, (0, 255, 255), -1)
                            else:
                                cv2.circle(image, pub_data[choose_next_point], 10, (0, 255, 255), -1)
                                # print("desire pub_data-======>",pub_data)
                                # if dd[1]<100 and dd[1]>40:
                                # rospy.loginfo("publisher----%s",str(dd))
                                self.desire_pub.publish(str(pub_data[choose_next_point]))
                                self.last_time_desire_pub_data=pub_data[choose_next_point]
                        else:
                            self.last_time_desire_pub_data=pub_data[choose_next_point]

                    cv2.namedWindow( 'YHLS_Black_HLS_Space', cv2.WINDOW_NORMAL )
                    cv2.imshow( 'YHLS_Black_HLS_Space', YHLS )

                    # cv2.namedWindow( 'Black_HLS_Space', cv2.WINDOW_NORMAL )
                    # cv2.imshow( 'Black_HLS_Space', thresh )

                    cv2.namedWindow( 'dilated_Black_HLS_Space', cv2.WINDOW_NORMAL )
                    cv2.imshow( 'dilated_Black_HLS_Space', dilated )

                    # cv2.namedWindow( 'Black_tile_edges', cv2.WINDOW_NORMAL )
                    # cv2.imshow( 'Black_tile_edges', Y_edges )

                    cv2.namedWindow( 'tile_pixel_frame', cv2.WINDOW_NORMAL )
                    cv2.imshow( 'tile_pixel_frame', image )

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
        desire_point=[[556,468],[554,71],[94,57],[80,468]]
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
