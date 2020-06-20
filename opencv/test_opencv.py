#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
# from tilling_robot.msg import uv
# from tilling_robot.msg import tileuv
from std_msgs.msg import Int32
import time
import numpy as np
import imutils
# from tilling_robot.msg import code_flags
# from tilling_robot.msg import sucker_tile_line
# from code_flags_sub import *
from std_msgs.msg import UInt8
""""
v4 sovles tile number random problem
"""
class DetectTile:

    def __init__(self):
        self.obj_buf = []
        self.desire_buf = []
        # self.sucker_obj_sub = rospy.Subscriber("/tilling_robot/sucker_center_obj_uv", uv, self.callback_obj)
        # self.sucker_desire_sub = rospy.Subscriber("/tilling_robot/sucker_center_desire_uv", uv, self.callback_desire)
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.rgb_image=None
        # self.tile_pub = rospy.Publisher("/tilling_robot/tile_uv", tileuv, queue_size=10)
        # self.sucker_line_pub = rospy.Publisher("/tilling_robot/sucker_line_uv", sucker_tile_line, queue_size=10)
    def callback(self,data):
        try:
            video_capture=self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.rgb_image = video_capture.copy()
        except CvBridgeError as e:
            print e
    def callback_obj(self,data):
        if len(self.obj_buf)==10:
            self.obj_buf=self.obj_buf[1:]
            self.obj_buf.append(data.uvinfo)
        else:
            self.obj_buf.append(data.uvinfo)
            print "obj_buf",self.obj_buf
    def callback_desire(self,data):
        if len(self.desire_buf)==10:
            self.desire_buf=self.desire_buf[1:]
            self.desire_buf.append(data.uvinfo)
        else:
            self.desire_buf.append(data.uvinfo)
    def convert_hsv(self,image):
        # bgr=image[...,::-1]
        return cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    def convert_hls(self,image):
        return cv2.cvtColor(image, cv2.COLOR_RGB2HLS)

    # image is expected be in RGB color space
    def select_rgb_white_yellow(self,image):
        # white color mask
        lower = np.uint8([0, 0, 0])
        upper = np.uint8([65, 255, 40])
        white_mask = cv2.inRange(image, lower, upper)
        # yellow color mask
        lower = np.uint8([190, 190, 0])
        upper = np.uint8([255, 255, 255])
        yellow_mask = cv2.inRange(image, lower, upper)
        # combine the mask
        mask = cv2.bitwise_or(white_mask, yellow_mask)
        masked = cv2.bitwise_and(image, image, mask=mask)
        return masked
    def select_rgb_black(self,image):
        # white color mask
        lower = np.uint8([0, 0, 0])
        upper = np.uint8([0, 0, 0])
        white_mask = cv2.inRange(image, lower, upper)
        # yellow color mask
        lower = np.uint8([0, 0, 0])
        upper = np.uint8([0, 0, 0])
        yellow_mask = cv2.inRange(image, lower, upper)
        # combine the mask
        mask = cv2.bitwise_or(white_mask, yellow_mask)
        masked = cv2.bitwise_and(image, image, mask=mask)
        return masked
    def convert_gray_scale(self,image):
        return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    def apply_smoothing(self,image, kernel_size=15):
        """
        kernel_size must be postivie and odd
        """
        return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)

    def detect_edges(self,image, low_threshold=50, high_threshold=150):
        return cv2.Canny(image, low_threshold, high_threshold)
    def select_white(self,image):
        converted = self.convert_hls(image)
        # converted = self.convert_hsv(image)

        # print "con-------",converted
        # white color mask
        #white
        lower = np.uint8([0, 200, 0])
        upper = np.uint8([255, 255, 255])
        # lower = np.uint8([10, 0, 100])
        # upper = np.uint8([40, 255, 255])
        white_mask = cv2.inRange(converted, lower, upper)
        #yellow color mask
        lower = np.uint8([0, 200, 0])
        upper = np.uint8([255, 255, 255])
        yellow_mask = cv2.inRange(converted, lower, upper)
        # combine the mask
        mask = cv2.bitwise_or(white_mask, yellow_mask)
        return cv2.bitwise_and(image, image, mask=mask)
    def select_yellow_1(self,image):
        # converted = self.convert_hls(image)
        converted = self.convert_hsv(image)#RGB
        lower = np.uint8([0, 0,29])
        upper = np.uint8([255, 255, 255])
        # lower = np.uint8([0, 0, 0])
        # upper = np.uint8([50, 50, 100])
        white_mask = cv2.inRange(converted, lower, upper)
        #yellow color mask
        # lower = np.uint8([0, 0, 0])
        # upper = np.uint8([50, 50, 100])
        lower = np.uint8([0, 0, 29])
        upper = np.uint8([255, 255, 255])
        # lower = np.uint8([0, 0, 0])
        # upper = np.uint8([50, 50, 100])
        yellow_mask = cv2.inRange(converted, lower, upper)
        # combine the mask
        mask = cv2.bitwise_or(white_mask, yellow_mask)
        return cv2.bitwise_and(image, image, mask=mask)
    def select_yellow(self,image):
        converted = self.convert_hls(image)
        # converted = self.convert_hsv(image)
        lower = np.uint8([0, 0, 0])
        upper = np.uint8([255,255,255])
        white_mask = cv2.inRange(converted, lower, upper)
        #yellow color mask
        lower = np.uint8([0, 0, 0])
        upper = np.uint8([255,255,255])
        yellow_mask = cv2.inRange(converted, lower, upper)
        # combine the mask
        mask = cv2.bitwise_or(white_mask, yellow_mask)
        return cv2.bitwise_and(image, image, mask=mask)
    def select_white_yellow(self,image):
        converted = self.convert_hls(image)
        # converted = self.convert_hsv(image)

        # print "con-------",converted
        # white color mask
        #white
        lower = np.uint8([0, 200, 0])
        upper = np.uint8([255, 255, 255])

        white_mask = cv2.inRange(converted, lower, upper)
        #yellow color mask
        lower = np.uint8([10, 0, 100])
        upper = np.uint8([40, 255, 255])
        yellow_mask = cv2.inRange(converted, lower, upper)
        # combine the mask
        mask = cv2.bitwise_or(white_mask, yellow_mask)
        return cv2.bitwise_and(image, image, mask=mask)

    def filter_region(self,image, vertices):
        """
        Create the mask using the vertices and apply it to the input image
        """
        mask = np.zeros_like(image)
        if len(mask.shape) == 2:
            cv2.fillPoly(mask, vertices, 255)
        else:
            cv2.fillPoly(mask, vertices, (255,) * mask.shape[2])  # in case, the input image has a channel dimension
        return cv2.bitwise_and(image, mask)

    def select_region(self,image,bottom_left_cols1,bottom_left_rows1,top_left_cols1,top_left_rows1,bottom_right_cols1,bottom_right_rows1,top_right_cols1,top_right_rows1):
        """
        It keeps the region surrounded by the `vertices` (i.e. polygon).  Other area is set to 0 (black).
        bottom_left_cols1=0.53
        bottom_left_rows1=0.70
        top_left_cols1=0.53
        top_left_rows1=0.28
        bottom_right_cols1=0.95
        bottom_right_rows1=0.70
        top_right_cols1=0.99
        top_right_rows1=0.28
        """
        # first, define the polygon by vertices
        rows, cols = image.shape[:2]
        bottom_left = [cols * bottom_left_cols1, rows * bottom_left_rows1]
        top_left = [cols * top_left_cols1, rows * top_left_rows1]
        bottom_right = [cols * bottom_right_cols1, rows * bottom_right_rows1]
        top_right = [cols *top_right_cols1, rows * top_right_rows1]
        # the vertices are an array of polygons (i.e array of arrays) and the data type must be integer
        vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
        return self.filter_region(image, vertices)

    def hough_lines(self,image):
        """
        `image` should be the output of a Canny transform.

        Returns hough lines (not the image with lines)
        """
        return cv2.HoughLinesP(image, rho=1, theta=np.pi / 180, threshold=20, minLineLength=20, maxLineGap=300)
    """
    line=[(-0.2867132867132867, 375.1958041958042), (-0.013986013986013986, 204.23776223776224), (-0.2857142857142857, 382.85714285714283), (-0.03292181069958848, 216.3127572016461)]
    get the best line 
    """

    def Get_same_slope(self,line):
        # result_list = []
        if line is not None:
            result_list = []
            for ii in xrange(len(line)):
                start = line[ii][0]
                for i in line:
                    # print i[0]
                    if abs(abs(i[0]) - abs(start)) <= 0.01 and abs(abs(i[0]) - abs(start)) != 0:
                        result_list.append(i)
                    else:
                        pass
            if len(result_list)!=0:
                slop_sum = 0
                slop_sum_final = 0
                intercept_sum_final = 0
                for i in result_list:
                    slop_sum += i[0]
                avg_slope = slop_sum / len(result_list)
                result_list_final = []
                for i in result_list:
                    if abs(i[0]) > abs(avg_slope):
                        result_list_final.append(i)
                    else:
                        pass
                for i in result_list_final:
                    slop_sum_final += i[0]
                    intercept_sum_final += i[1]
                if len(result_list_final)!=0:

                    return (slop_sum_final / len(result_list_final), intercept_sum_final / len(result_list_final))
                else:
                    pass
            else:
                pass
        else:
            pass

    def average_slope_intercept(self,lines):
        left_lines = []  # (slope, intercept)
        left_weights = []  # (length,)
        right_lines = []  # (slope, intercept)
        right_weights = []  # (length,)
        all_lines = []
        zero_lines = []
        zero_weights = []
        for line in lines:
            for x1, y1, x2, y2 in line:
                # if x2 == x1:
                #     continue  # ignore a vertical line
                # print "x1, y1, x2, y2",x1, y1, x2, y2
                slope = (y2 - y1) * 0.1 / ((x2 - x1) * 0.1)

                intercept = y1 - slope * x1
                length = np.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
                # print "slope,intercept,length",slope,intercept,length
                all_lines.append((slope, intercept))
            all_line = self.Get_same_slope(all_lines)
            # right_lane = self.Get_same_slope(right_lines)
            # print "left_lane, right_lane", left_lane, right_lane
        return all_line

    def make_line_points(self,y1, y2, line):
        """
        Convert a line represented in slope and intercept into pixel points
        """
        if line is None:
            return None

        slope, intercept = line
        # try:
        # make sure everything is integer as cv2.line requires it
        if slope != 0 and slope != (float("-inf")) and slope != float("inf"):
            x1 = int((y1 - intercept) * 0.1 / (slope * 0.1))
            x2 = int((y2 - intercept) * 0.1 / (slope * 0.1))
            y1 = int(y1)
            y2 = int(y2)

            return ((x1, y1), (x2, y2))
        elif slope == 0:
            print "slope is zero"
            return ((300, int(intercept)), (900, int(intercept)))
        else:
            pass


    def tile_sucker_lines(self,image, lines):
        if image is not None:
            if self.average_slope_intercept(lines) != 0:
                all_line = self.average_slope_intercept(lines)
                y1 = image.shape[0] #* 0.5  # bottom of the image
                y2 = y1 * 0.1  # slightly lower than the middle
                line_info= self.make_line_points(y1, y2, all_line)

                return line_info,all_line
            else:
                pass
    def Judge_isnot_same_tile(self,latest_central,now_central):
        if abs(latest_central[0]-now_central[0])<=20 and abs(latest_central[1]-now_central[1])<=20:
            return 1
        else:
            return 0

    def Sort_tile_feature(self,approx):
        result = []
        new_approx = []
        for i in approx:
            new_approx.append(i[0][0])
        new_approx=list(set(new_approx))#distinct
        # print new_approx
        # temp = []
        # for i in new_approx:
        #     temp.append(i[0])
        new_approx.sort(reverse=True)
        for i in new_approx:  # 升序排列
            for ii in approx:
                if ii[0][0] == i:
                    result.append(ii[0])
        # print result
        return result
    def Sort_tile_mass_num(self,resultuv):
        tempxy = []
        tempy = []
        resultmp = []
        lastresult = []
        tmp = []
        newreslutuv = []
        # print resultuv
        for i in range(len(resultuv)):
            if resultuv[i][0][0] == i + 1:
                newreslutuv.append(resultuv[i])
        for i in newreslutuv:
            # if i[0][0]
            tmp.append(i[1])
        tempxy = sorted(tmp, reverse=True, key=lambda k: [k[1], k[0]])
        print tempxy
        for i in tempxy:
            for ii in newreslutuv:
                if i[0] == ii[1][0] and i[1] == ii[1][1]:
                    resultmp.append(ii)
        for i in range(len(resultmp)):
            woqu = (resultmp[i][1:])
            # print "woqutype",type(woqu)
            woqu.insert(0, [i + 1])
            # print woqu
            lastresult.append(woqu)
        return lastresult
    def Draw_triangle(self,contours,rgb,obj_desire):
        ##################
        DELAY = 0.02
        USE_CAM = 1
        IS_FOUND = 0
        count=0#count feature tile numbers
        cnt=0



        central_list=[]
        # uvuv=uv()
        # tile_uv=tileuv()
        ##################
        _width  = 480.0
        _height = 640.0
        _margin = 0.0
        corners = np.array(
            [
                [[  		_margin, _margin 			]],
                [[ 			_margin, _height + _margin  ]],
                [[ _width + _margin, _height + _margin  ]],
                [[ _width + _margin, _margin 			]],
            ]
        )

        pts_dst = np.array( corners, np.float32 )
        latest_central=(0,0)
        resultuv = []
        for cont in contours:

            """
            #1,num,2,centeral point 3,for angular point uv ,4,clockwise direction
            #caculating Area for tile selected just one tile
            """
            # print "cont----------", cont
            # 获取轮廓长度
            arc_len = cv2.arcLength(cont, True)
            # 多边形拟合
            approx = cv2.approxPolyDP(cont, 0.01 * arc_len, True)
            # print "cv2.contourArea(cont)",cv2.contourArea(cont)
            print "approx",len(np.array(approx).reshape(-1,2))
            # if cv2.contourArea(cont) > 0:# and cv2.contourArea(cont) < 1000000:
            if cv2.contourArea(cont) > 30:
                # print()
                if (len(approx) == 4):
                    # print "a"
                    IS_FOUND = 1
                    M = cv2.moments(cont)
                    # 获取图像质心坐标
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    now_central = (cX, cY)
                    if self.Judge_isnot_same_tile(latest_central, now_central) != 1:
                        count += 1
                    # cv2.putText(rgb, str(count), (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
                    # print "CX,CY", [cX, cY]
                    central_list.append([cX, cY])
                    pts_src = np.array(approx, np.float32)
                    # print "pts_src", pts_src
                    cv2.circle(rgb, (cX, cY), 5, (0, 0, 0), -1)
                    # print approx.tolist()
                    angular_point = []
                    print "approx", approx.tolist()
                    new_approx=self.Sort_tile_feature(approx.tolist())
                    new_approx=sorted(new_approx, reverse=True, key=lambda k: [k[1], k[0]])
                    # new_approx=approx

                    print "new_approx",new_approx
                    for i in range(len(new_approx)):
                        if i == 0:
                            cv2.circle(rgb, (new_approx[i][0], new_approx[i][1]), 5, (20, 60, 220), -1)
                            angular_point.append([new_approx[i][0], new_approx[i][1]])

                            cv2.putText(rgb, str(i), (new_approx[i][0], new_approx[i][1]),
                                        cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 0), 1)
                        else:
                            cv2.circle(rgb, (new_approx[i][0], new_approx[i][1]), 5, (0, 255, 0), -1)
                            angular_point.append([new_approx[i][0], new_approx[i][1]])

                            cv2.putText(rgb, str(i), (new_approx[i][0], new_approx[i][1]),
                                        cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 0), 1)

                    resultuv.append([[count],[cX, cY], angular_point])
                    # draw trangle in image

                    h, status = cv2.findHomography(pts_src, pts_dst)
                    out = cv2.warpPerspective(rgb, h, (int(_width + _margin * 2), int(_height + _margin * 2)))

                    cv2.drawContours(rgb, [approx], -1, (0, 255, 255), 3)
                    # cv2.drawContours(rgb, [approx], -1, (20*count, 255, 0), -1, cv2.LINE_AA)
                    print "all info for tile------", resultuv
                    print "angular_point",angular_point
                    cv2.line(rgb, ((angular_point[0][0]+angular_point[2][0])/2,(angular_point[0][1]+angular_point[2][1])/2), (cX,cY), (255,0,0), 3)
                    cv2.line(rgb, (
                    (angular_point[1][0] + angular_point[3][0]) / 2, (angular_point[1][1] + angular_point[3][1]) / 2),
                             (cX, cY), (255, 0, 0), 3)
                    # print "Now tile id",count
                    # if count == 1:
                    #     tile_uv.tile_id = count
                    #     tile_uv.obj_desire = obj_desire
                    #     tile_uv.cen_uv.uvinfo = [cX, cY]
                    #     tile_uv.f1th_uv.uvinfo = angular_point[0]
                    #     tile_uv.s2th_uv.uvinfo = angular_point[1]
                    #     tile_uv.t3th_uv.uvinfo = angular_point[2]
                    #     tile_uv.f4th_uv.uvinfo = angular_point[3]
                    #     self.tile_pub.publish(tile_uv)

                    latest_central = now_central

                else:
                    pass
                # count += 1
                # cnt += 11
        # return rgb.copy()
        return resultuv,rgb.copy()
    def draw_triangle_numbers(self,contours,rgb,obj_desire):
        # uvuv=uv()
        # tile_uv=tileuv()
        alltile_uvlist,newrgb=self.Draw_triangle(contours,rgb,obj_desire)
        newtileuv=self.Sort_tile_mass_num(alltile_uvlist)
        for i in newtileuv:
            # print "i",i
            cv2.putText(newrgb, str(i[0][0]), (i[1][0], i[1][1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
            # tile_uv.tile_id = i[0][0]
            # tile_uv.obj_desire = obj_desire
            # tile_uv.cen_uv.uvinfo = [i[1][0], i[1][1]]
            # tile_uv.f1th_uv.uvinfo = i[2][0]
            # tile_uv.s2th_uv.uvinfo = i[2][1]
            # tile_uv.t3th_uv.uvinfo = i[2][2]
            # tile_uv.f4th_uv.uvinfo = i[2][3]
            # self.tile_pub.publish(tile_uv)
        return newrgb.copy()

    # def pub_empty_uv_info(self,tile_id,obj_desire):
    #     uvuv=uv()
    #     tile_uv=tileuv()
    #     tile_uv.tile_id = tile_id
    #     tile_uv.obj_desire = obj_desire
    #     tile_uv.cen_uv.uvinfo = [0,0]
    #     tile_uv.f1th_uv.uvinfo = [0,0]
    #     tile_uv.s2th_uv.uvinfo = [0,0]
    #     tile_uv.t3th_uv.uvinfo = [0,0]
    #     tile_uv.f4th_uv.uvinfo = [0,0]
    #     self.tile_pub.publish(tile_uv)
    def process_rgb_object_image(self,rgb_image):

        MORPH = 7
        CANNY = 250
        ##################
        rgb=rgb_image
        image=rgb_image
        if rgb_image is not None:


            """'
            Select Blue Desire position
            tile_id=1,fixed point id
            obj_desire="o" object
            """
            YHLS=self.select_yellow(rgb)
            # print "YHLS",YHLS
            # Y_gray = self.convert_gray_scale(rgb)
            # Y_smooth = self.apply_smoothing(Y_gray,1)
            # Y_edges = self.detect_edges(Y_smooth)
            # Y_kernel = cv2.getStructuringElement( cv2.MORPH_RECT, ( MORPH, MORPH ) )
            # Y_closed = cv2.morphologyEx( Y_edges.copy(), cv2.MORPH_CLOSE, Y_kernel )

            resized = imutils.resize(image, width=640)
            ratio = image.shape[0] / float(resized.shape[0])
            gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            thresh = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)[1]


            lsd = cv2.createLineSegmentDetector(0, _scale=0.5) 
            dlines = lsd.detect(thresh) 
            for dline in dlines[0]:  
                x0 = int(round(dline[0][0]))     
                y0 = int(round(dline[0][1]))    
                x1 = int(round(dline[0][2]))     
                y1 = int(round(dline[0][3]))     
                cv2.line(rgb, (x0, y0), (x1,y1), (0,0,255), 2, cv2.LINE_AA) 

            Y_contours= cv2.findContours( thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )



            # resized = imutils.resize(image, width=640)
            # ratio = image.shape[0] / float(resized.shape[0])
            # gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
            # blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            # thresh = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)[1]
            # cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            Y_contours = imutils.grab_contours(Y_contours)
            # print "Y_h",Y_h,Y_contours
            if len(Y_contours)!=0:
                # print("haha")
                rgb = self.draw_triangle_numbers(Y_contours, rgb,'o')
            else:
                print "There is no tile0,you need put one black tile"
                # self.pub_empty_uv_info(0, 'o')
            cv2.circle(rgb, (316,251), 10, (20, 100, 220), -2)
            cv2.putText(rgb, 'center', (316,251),
                        cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 0), 1)
            if len(self.obj_buf)!=0:
                cv2.circle(rgb, (int(self.obj_buf[-1][0]), int(self.obj_buf[-1][1])), 10, (100, 100, 220), -2)
                cv2.putText(rgb, 'oc', (int(self.obj_buf[-1][0]), int(self.obj_buf[-1][1])),
                            cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 0), 1)
            if len(self.desire_buf)!=0:
                cv2.circle(rgb, (int(self.desire_buf[-1][0]), int(self.desire_buf[-1][1])), 10, (200, 200, 220), -2)
                cv2.putText(rgb, 'dc', (int(self.desire_buf[-1][0]), int(self.desire_buf[-1][1])),
                            cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 0), 1)
            # """
            # HLS SPACE
            # """
            # HLSDOUBLE=self.convert_hls(rgb)
            # cv2.namedWindow( 'HLSDOUBLE_Space', cv2.WINDOW_NORMAL )
            # cv2.imshow( 'HLSDOUBLE_Space', HLSDOUBLE )
            #
            cv2.namedWindow( 'Black_HLS_Space', cv2.WINDOW_NORMAL )
            cv2.imshow( 'Black_HLS_Space', thresh )

            # cv2.namedWindow( 'Black_tile_edges', cv2.WINDOW_NORMAL )
            # cv2.imshow( 'Black_tile_edges', Y_edges )

            cv2.namedWindow( 'tile_pixel_frame', cv2.WINDOW_NORMAL )
            cv2.imshow( 'tile_pixel_frame', rgb )

            cv2.waitKey(8)

            # # 再将opencv格式额数据转换成ros image格式的数据发布
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(rgb_image, "bgr8"))
            except CvBridgeError as e:
                print e
        # return central_list
        """
        Row,x
        col,v
        For example:
        point_x0=235
        point_x1=429
        """
    def calu_two_points(self,slop_intercept,point_x0,point_x1):
        reslut=[]
        if slop_intercept[0]!=0:
            v0=slop_intercept[0]*point_x0+slop_intercept[1]
            v1=slop_intercept[0]*point_x1+slop_intercept[1]
            reslut.append([point_x0,v0])
            reslut.append([point_x1, v1])
            return reslut
        else:
            reslut.append([point_x0,slop_intercept[1]])
            reslut.append([point_x1, slop_intercept[1]])
            return reslut
    def process_rgb_desire_image(self,rgb_image):
        color1 = [0, 0, 255]
        color2 = [255, 0, 0]
        color3 = [0, 255, 0]
        thickness = 5

        """
        region selecting
        row 行
        """
        bottom_left_cols1 = 0.33
        bottom_left_rows1 = 0.15
        top_left_cols1 = 0.33
        top_left_rows1 = 0.08
        bottom_right_cols1 = 0.75
        bottom_right_rows1 = 0.15
        top_right_cols1 = 0.75
        top_right_rows1 = 0.08
        sucker_line_uv = sucker_tile_line()
        # uvuv = uv()
        MORPH = 7
        CANNY = 250
        ##################
        rgb=rgb_image

        if rgb_image is not None:


            """'
            Select Blue Desire position
            tile_id=1,fixed point id
            obj_desire="o" object
            """
            YHLS=self.select_yellow(rgb)
            # print "YHLS",YHLS
            Y_gray = self.convert_gray_scale(YHLS)
            Y_smooth = self.apply_smoothing(Y_gray,1)
            Y_edges = self.detect_edges(Y_smooth)
            New_edges = Y_edges.copy()


            Y_kernel = cv2.getStructuringElement( cv2.MORPH_RECT, ( MORPH, MORPH ) )
            Y_closed = cv2.morphologyEx( Y_edges.copy(), cv2.MORPH_CLOSE, Y_kernel )
            _,Y_contours, Y_h = cv2.findContours( Y_closed.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )
            # print "Y_h",Y_h,Y_contours
            if len(Y_contours)!=0:
                rgb = self.draw_triangle_numbers(Y_contours, rgb,'d')
            else:
                print "There is no tile0,you need put one black tile"
                # self.pub_empty_uv_info(0, 'o')
            cv2.circle(rgb, (316,251), 10, (20, 100, 220), -2)
            cv2.putText(rgb, 'center', (316,251),
                        cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 0), 1)
            if len(self.obj_buf)!=0:
                cv2.circle(rgb, (int(self.obj_buf[-1][0]), int(self.obj_buf[-1][1])), 10, (100, 100, 220), -2)
                cv2.putText(rgb, 'oc', (int(self.obj_buf[-1][0]), int(self.obj_buf[-1][1])),
                            cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 0), 1)
            if len(self.desire_buf)!=0:
                cv2.circle(rgb, (int(self.desire_buf[-1][0]), int(self.desire_buf[-1][1])), 10, (200, 200, 220), -2)
                cv2.putText(rgb, 'dc', (int(self.desire_buf[-1][0]), int(self.desire_buf[-1][1])),
                            cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 0), 1)
            """
            bottom_left_cols1=0.53
            bottom_left_rows1=0.70
            top_left_cols1=0.53
            top_left_rows1=0.28
            bottom_right_cols1=0.45
            bottom_right_rows1=0.70
            top_right_cols1=0.45
            top_right_rows1=0.28
            """
            # region_up = self.select_region(Y_edges, 0.44, 0.80, 0.44, 0.15, 0.70, 0.80, 0.70, 0.15)
            region_up = self.select_region(Y_edges, bottom_left_cols1, bottom_left_rows1, top_left_cols1, top_left_rows1,bottom_right_cols1, bottom_right_rows1, top_right_cols1, top_right_rows1)
            region_up_line = self.hough_lines(region_up)
            left_line_info,slop_intercept = self.tile_sucker_lines(rgb, region_up_line)

            if left_line_info!=None:
                cv2.line(rgb, left_line_info[0], left_line_info[1], color1, thickness)

                sucker_line_uv.sucker_tile_uv0.uvinfo = [left_line_info[0][0], left_line_info[0][1]]
                sucker_line_uv.sucker_tile_uv1.uvinfo= [left_line_info[1][0], left_line_info[1][1]]

                sucker_line_uv.sucker_tile_slope = slop_intercept[0]
                sucker_line_uv.sucker_tile_intercept = slop_intercept[1]
                # self.sucker_line_pub.publish(sucker_line_uv)
                print "slop_intercept--------------", slop_intercept
            else:
                pass
            """
            line detect
            """
            cv2.namedWindow( 'region_up', cv2.WINDOW_NORMAL )
            cv2.imshow( 'region_up', region_up )

            # cv2.namedWindow( 'sucker_line', cv2.WINDOW_NORMAL )
            # cv2.imshow( 'sucker_line', region_up_line )
            # """
            # HLS SPACE
            # """
            # HLSDOUBLE=self.convert_hls(rgb)
            # cv2.namedWindow( 'HLSDOUBLE_Space', cv2.WINDOW_NORMAL )
            # cv2.imshow( 'HLSDOUBLE_Space', HLSDOUBLE )
            #
            cv2.namedWindow( 'Black_HLS_Space', cv2.WINDOW_NORMAL )
            cv2.imshow( 'Black_HLS_Space', YHLS )
            cv2.namedWindow( 'Black_tile_edges', cv2.WINDOW_NORMAL )
            cv2.imshow( 'Black_tile_edges', Y_edges )

            cv2.namedWindow( 'tile_pixel_frame', cv2.WINDOW_NORMAL )
            cv2.imshow( 'tile_pixel_frame', rgb )

            cv2.waitKey(8)

            # # 再将opencv格式额数据转换成ros image格式的数据发布
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(rgb_image, "bgr8"))
            except CvBridgeError as e:
                print e
        # return central_list


def main():
    try:
        # 初始化ros节点
        rospy.init_node("cv_bridge_test")
        rospy.loginfo("Starting cv_bridge_test node")
        k=DetectTile()
        # code_flag_sub=Codeflags()

        """
        we use flag to choose different state
        0,Use for one feature IBVS
        1,For detecting object tile
        2,Use for detecting two feature with IBVS
        3,Use for two features IBVS
        """
        # flag_data = code_flags()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            k.process_rgb_object_image(k.rgb_image)
            # try:
            #     if len(code_flag_sub.object_detect_id_buf)!=0:
            #         # print "code_flag_sub.object_detect_id_buf[-1]",code_flag_sub.object_detect_id_buf[-1]
            #         if code_flag_sub.object_detect_id_buf[-1]==1:
            #             k.process_rgb_object_image(k.rgb_image)
            #             # cen=k.process_rgb_image(k.rgb_image)
            #             # print "cenpixel\n",cen
            #             time.sleep(1.5)
            #     else:
            #         print "please wait open object vision flag----"
            #     if len(code_flag_sub.desire_detect_id_buf)!=0:
            #         if code_flag_sub.desire_detect_id_buf[-1] == 1:
            #             k.process_rgb_desire_image(k.rgb_image)
            #             # cen=k.process_rgb_image(k.rgb_image)
            #             # print "cenpixel\n",cen
            #             # print "haha"
            #             time.sleep(1.5)
            #     else:
            #         print "please wait open desire vision flag----"
            rate.sleep()
            # except:
            #     print "no sucking tile----"
       # rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node."
        cv2.destroyAllWindows()
if __name__=="__main__":
    main()
