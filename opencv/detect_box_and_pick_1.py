#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time
import numpy as np
import imutils
from pylsd.lsd import lsd


class DetectTile:
    def __init__(self):
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.rgb_image=None
        self.avg_data_x=[]
        self.avg_data_y=[]
        self.cross_point_data=[]
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
    def select_yellow(self,image):
        converted = self.convert_hls(image)
        # converted = self.convert_hsv(image)
        lower = np.uint8([0, 120, 0])
        upper = np.uint8([255,255,255])
        white_mask = cv2.inRange(converted, lower, upper)
        #yellow color mask
        lower = np.uint8([0, 120, 0])
        upper = np.uint8([255,255,255])
        yellow_mask = cv2.inRange(converted, lower, upper)
        # combine the mask
        mask = cv2.bitwise_or(white_mask, yellow_mask)
        return cv2.bitwise_and(image, image, mask=mask)
    def corner(self,rgb_image):
        img=rgb_image
        cross_point=[]
        p=(0,0)
        rgb=rgb_image
        self.cross_point_data=[]
        self.avg_data_x=[]
        self.avg_data_y=[]
        if img is not None:

            
            # img = cv2.imread(filename)
            rgb = img.copy()

            YHLS=self.select_yellow(rgb)
            # print "YHLS",YHLS
            # Y_gray = self.convert_gray_scale(rgb)
            # Y_smooth = self.apply_smoothing(Y_gray,1)
            # Y_edges = self.detect_edges(Y_smooth)
            # Y_kernel = cv2.getStructuringElement( cv2.MORPH_RECT, ( MORPH, MORPH ) )
            # Y_closed = cv2.morphologyEx( Y_edges.copy(), cv2.MORPH_CLOSE, Y_kernel )

            # resized = imutils.resize(YHLS, width=640)
            # ratio = img.shape[0] / float(resized.shape[0])
            # gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
            # blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            # thresh = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)[1]
            gray = cv2.cvtColor(YHLS, cv2.COLOR_BGR2GRAY)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(11, 11))
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
                    
                        # cv2.circle(rgb, dd, 10, (0, 255, 0), -1)
                
                    cv2.line(rgb, (x0, y0), (x1,y1), (0,0,255), 2, cv2.LINE_AA)
                for i in range(len(cross_point)):
                    if i+1<len(cross_point):
                        dd=self.cross_point(cross_point[i],cross_point[i+1])
                        # if dd[0]<100:
                        if dd[0]>0 and dd[1]>0:
                            
                            cv2.circle(rgb, dd, 10, (255, 0, 0), -1)
                            if dd[0]<545 and dd[1]<450:
                                self.avg_data_x.append(dd[0])
                                self.avg_data_y.append(dd[1])
                                self.cross_point_data.append(dd)
                                print("dd",dd)
                        

            print(np.mean(self.avg_data_x),np.mean(self.avg_data_y))
            if len(self.cross_point_data)!=0:
                for i in range(len(self.cross_point_data)):
                    if self.cross_point_data[i][0]< np.mean(self.avg_data_x) and self.cross_point_data[i][1]< np.mean(self.avg_data_y):
                        cv2.circle(rgb, self.cross_point_data[i], 10, (0, 0, 255), -1)
                        print("desire point",self.cross_point_data[i])
            cv2.namedWindow( 'YHLS_Black_HLS_Space', cv2.WINDOW_NORMAL )
            cv2.imshow( 'YHLS_Black_HLS_Space', YHLS )

            # cv2.namedWindow( 'Black_HLS_Space', cv2.WINDOW_NORMAL )
            # cv2.imshow( 'Black_HLS_Space', thresh )

            cv2.namedWindow( 'dilated_Black_HLS_Space', cv2.WINDOW_NORMAL )
            cv2.imshow( 'dilated_Black_HLS_Space', dilated )

            # cv2.namedWindow( 'Black_tile_edges', cv2.WINDOW_NORMAL )
            # cv2.imshow( 'Black_tile_edges', Y_edges )

            cv2.namedWindow( 'tile_pixel_frame', cv2.WINDOW_NORMAL )
            cv2.imshow( 'tile_pixel_frame', rgb )

            cv2.waitKey(8)

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
            except CvBridgeError as e:
                print(e)

def main():
    try:
        # 初始化ros节点
        rospy.init_node("cv_bridge_test_1")
        rospy.loginfo("Starting cv_bridge_test node")
        k=DetectTile()
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            # k.process_rgb_image(k.rgb_image)
            k.corner(k.rgb_image)
            # k.process_rgb_object_image(k.rgb_image)
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
