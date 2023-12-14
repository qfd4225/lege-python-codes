from pyorbbecsdk import Pipeline, FrameSet
from pyorbbecsdk import Config
from pyorbbecsdk import OBSensorType, OBFormat
from pyorbbecsdk import OBError
from pyorbbecsdk import VideoStreamProfile
from pyorbbecsdk import *
import cv2
import numpy as np
from utils import frame_to_bgr_image
import os
import sys
import argparse
import cv2.aruco as aruco
import math
from PIL import Image
import time
from sklearn.linear_model import LinearRegression
#from pymycobot import ElephantRobot


# class Bot:
#     def __init__(self):
#         self.elephant_client = ElephantRobot("10.17.235.83", 5001)
#         self.elephant_client.start_client()

#     def bot_run_coords(self, x, y, z, rx, ry, rz):
#         # 杩愬姩鍒扮瑳鍗″皵鍧愭爣
#         self.elephant_client.write_coords([x, y, z, rx, ry, rz], 1500)
#         self.elephant_client.command_wait_done()
#         self.elephant_client.wait(1)

#     def bot_run_angles(self, j0, j1, j2,j3, j4, j5):
#         self.elephant_client.write_angles([j0, j1, j2,j3 ,j4 ,j5], 1000)
#         self.elephant_client.command_wait_done()
#         self.elephant_client.wait(1)
        
#     def bot_catch(self):
#         self.elephant_client.set_digital_out(0,1)
#         self.elephant_client.wait(5)

#     def bot_catch_over(self):
#         self.elephant_client.set_digital_out(0,0)
#         self.elephant_client.wait(1)
    
#     def bot_run_coord(self, xyz, coord):
#         self.elephant_client.write_coord(xyz, coord, 1500)
#         self.elephant_client.command_wait_done()
#         self.elephant_client.wait(1)
        
#     def get_coord(self):
#         xyz=self.elephant_client.get_coords()
#         self.elephant_client.command_wait_done()
#         self.elephant_client.wait(1)
#         return xyz


# 广角参数
fx = 360.454 
fy = 360.454 
cx = 315.851 
cy = 238.612


# 同一套
fx_d = fx
fy_d = fy
cx_d = cx
cy_d = cy



k1=0.318086 
k2=-0.449834
k3=0.189244
k4=0 
k5=0
k6=0
p1=0.000677395
p2=-0.00032566




def get_caliboard_corners(imagecolor,rows,cols):
    # 读取标定板图像
    #image = imagecolor
    gray = cv2.cvtColor(imagecolor, cv2.COLOR_BGR2GRAY)

    imgdist=undistort_image(imagecolor)
    graydist = cv2.cvtColor(imgdist, cv2.COLOR_BGR2GRAY)
    # 查找标定板角点
    ret, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
    ret1, corners1 = cv2.findChessboardCorners(graydist, (cols, rows), None)

    # 如果找到了角点
    if ret1 == True and ret == True:       
        # 进行亚像素级别的精细化调整
        criteria11 = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
        cv2.cornerSubPix(graydist, corners1, (3, 3), (-1, -1), criteria11)      

        # 进行亚像素级别的精细化调整
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
        cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), criteria)      

        corners_3d = []
        for i in range(len(corners1)):
            
            x, y = corners[i].ravel()
            x1,y1 = corners1[i].ravel()

            cv2.circle(imagecolor, (int(x), int(y)), 5, (0, 255, 255), 1,cv2.LINE_8)     
            cv2.circle(imgdist, (int(x1), int(y1)), 5, (0, 255, 255), 1,cv2.LINE_8) 
               
            
    return imgdist,imagecolor,ret
    


def cal3d1(p_x,p_y,deep):

    X = (p_x - cx_d) * deep / fx_d  # cx = self.camera_matrix
    Y = (p_y - cy_d) * deep / fy_d
    wd_p = [X, Y, deep]
    return wd_p

ESC_KEY = 27

sys.path.append('/home/lg/cjt_project/agv/mmdet/mmdeploy/build/lib')

pattern_size = (6, 4) 


# 构造参数解析器

aruco_type = "DICT_5X5_100"
ARUCO_DICT = {"DICT_4X4_50": cv2.aruco.DICT_4X4_50, "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
                   "DICT_4X4_250": cv2.aruco.DICT_4X4_250, "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
                   "DICT_5X5_50": cv2.aruco.DICT_5X5_50, "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
                   "DICT_5X5_250": cv2.aruco.DICT_5X5_250, "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
                   "DICT_6X6_50": cv2.aruco.DICT_6X6_50, "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
                   "DICT_6X6_250": cv2.aruco.DICT_6X6_250, "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
                   "DICT_7X7_50": cv2.aruco.DICT_7X7_50, "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
                   "DICT_7X7_250": cv2.aruco.DICT_7X7_250, "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
                   "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
                   "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
                   "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
                   "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
                   "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11}

num_chessboards = 1 

def undistort_image(img):
    # 读取输入图像 

    # 设置相机矩阵和畸变系数
    camera_matrix = np.array([[fx_d, 0, cx_d],[0, fy_d, cy_d],[0, 0, 1]], dtype=np.float64)
    
    dist_coeffs = np.array([k1, k2, p1, p2, k3, k4, k5, k6], dtype=np.float64)

    # 进行相机畸变校正
    undistorted_img = cv2.undistort(img, camera_matrix, dist_coeffs)

    return undistorted_img

def save_depth_frame(frame, time): # : DepthFrame
    if frame is None:
        return
    print(frame.get_depth_scale())
    width = frame.get_width()
    height = frame.get_height()
    timestamp = frame.get_timestamp()
    scale = frame.get_depth_scale()
    data = np.frombuffer(frame.get_data(), dtype=np.uint16)
    data = data.reshape((height, width))
    data = data.astype(np.float32) * scale
    data = data.astype(np.float32) 
    data = data.astype(np.uint16)
    
    data = data.reshape(height, width, 1)
    
    
    save_image_dir = os.path.join(os.getcwd(), "save_images_code")
    if not os.path.exists(save_image_dir):
        os.mkdir(save_image_dir)
    png_file = save_image_dir + "/d{}.png".format(time)
    cv2.imwrite(png_file, data)


def save_color_frame(frame, time):# : ColorFrame
    if frame is None:
        return
    width = frame.get_width()
    height = frame.get_height()
    timestamp = frame.get_timestamp()
    save_image_dir = os.path.join(os.getcwd(), "save_images_code")
    if not os.path.exists(save_image_dir):
        os.mkdir(save_image_dir)
    filename = save_image_dir + "/c{}.png".format(time)
    image = frame_to_bgr_image(frame)
    if image is None:
        print("failed to convert frame to image")
        return
    cv2.imwrite(filename, image)

def save_depth_frame_t(frame, time): # : DepthFrame
    if frame is None:
        return
    print(frame.get_depth_scale())
    width = frame.get_width()
    height = frame.get_height()
    timestamp = frame.get_timestamp()
    scale = frame.get_depth_scale()
    data = np.frombuffer(frame.get_data(), dtype=np.uint16)
    data = data.reshape((height, width))
    data = data.astype(np.float32) * scale
    data = data.astype(np.float32) 
    data = data.astype(np.uint16)
    
    data = data.reshape(height, width, 1)
    
    
    save_image_dir = os.path.join(os.getcwd(), "test_pic")
    if not os.path.exists(save_image_dir):
        os.mkdir(save_image_dir)
    png_file = save_image_dir + "/d_t.png"
    cv2.imwrite(png_file, data)


def save_color_frame_t(frame, time):# : ColorFrame
    if frame is None:
        return
    width = frame.get_width()
    height = frame.get_height()
    timestamp = frame.get_timestamp()
    save_image_dir = os.path.join(os.getcwd(), "test_pic")
    if not os.path.exists(save_image_dir):
        os.mkdir(save_image_dir)
    filename = save_image_dir + "/c_t.png"
    image = frame_to_bgr_image(frame)
    if image is None:
        print("failed to convert frame to image")
        return
    cv2.imwrite(filename, image)

def main():
    # 验证 OpenCV 是否支持提供的 ArUCo 标签


    save_data_dir1 = os.path.join(os.getcwd(), "save_images_code")            
    filename1 = save_data_dir1 + "/coordinate.txt"
    data1 = open(filename1,'w')
    config = Config()
    pipeline = Pipeline()
    try:
        profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        assert profile_list is not None
        try:
            depth_profile = profile_list.get_video_stream_profile(640, 0, OBFormat.Y11, 30)
        except OBError as e:
            print("Error: ", e)
            print("++++++++------------------------------++++++++")
            depth_profile = profile_list.get_default_video_stream_profile()
        assert depth_profile is not None
        print("depth profile: ", type(depth_profile))
        config.enable_stream(depth_profile)
    except Exception as e:
        print(e)
        return  
    try:
        profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        try:
            color_profile: VideoStreamProfile = profile_list.get_video_stream_profile(640, 0, OBFormat.UYVY, 30)
        except OBError as e:
            print(e)
            print("++++++++++++++++++++++++++++++++++++++++++++++++++++")
            color_profile = profile_list.get_default_video_stream_profile()
        config.enable_stream(color_profile)
        config.set_align_mode(OBAlignMode.HW_MODE)
    except Exception as e:
        print(e)
        return

    height, width = 480, 640
    red_color = (0, 0, 255)  # BGR格式，红色为(0, 0, 255)
    red_image = np.full((height, width, 3), red_color, dtype=np.uint8)

    pipeline.start(config)
    temp = 0
    while True:
        try:
            frames: FrameSet = pipeline.wait_for_frames(10000)
            if frames is None:
                continue
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if color_frame is None:
                continue
            # covert to RGB format
            color_image = frame_to_bgr_image(color_frame)
            
            color_copy = color_image.copy()
            color_ccopy=color_copy.copy()
            
            color_undist=undistort_image(color_ccopy)
            gray_undist=cv2.cvtColor(color_undist, cv2.COLOR_BGR2GRAY)


            arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
            arucoParams = cv2.aruco.DetectorParameters()
            (corners, ids, rejected) = cv2.aruco.detectMarkers(gray_undist, arucoDict, parameters=arucoParams)
            print(rejected)
            #(corners, ids, rejected) = cv2.aruco.detectMarkers(gray_undist, arucoDict, parameters=arucoParams)
            if len(corners) == 4:
                point_id_ = 0
                for (markerCorner, markerID) in zip(corners, ids):
                    corners_ = markerCorner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners_
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))
                    cv2.line(color_undist, topLeft, topRight, (0, 255, 0), 2)
                    cv2.line(color_undist, topRight, bottomRight, (0, 255, 0), 2)
                    cv2.line(color_undist, bottomRight, bottomLeft, (0, 255, 0), 2)
                    cv2.line(color_undist, bottomLeft, topLeft, (0, 255, 0), 2)
                    cv2.putText(color_undist, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    for corner_pt in markerCorner.squeeze():
                        corner_pt = corner_pt.squeeze()
                        cv2.putText(color_undist, str(point_id_), (int(corner_pt[0]), int(corner_pt[1])),cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255, 0, 0), 1)
                        point_id_ += 1
                    
            cv2.imshow("Chessboard Detection-color_copy", color_copy)
            
            
            if color_image is None:
                print("failed to convert frame to image")
                continue
            #cv2.imshow("Color Viewer", color_image)
            cv2.imshow("Chessboard Detection-gray_undist", color_undist)
            if depth_frame is None:
                continue
            
            width = depth_frame.get_width()
            height = depth_frame.get_height()
            scale = depth_frame.get_depth_scale()
            
            depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
            depth_data = depth_data.reshape((height, width))

            depth_image = cv2.normalize(depth_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_image = cv2.applyColorMap(depth_image, cv2.COLORMAP_JET)

            cv2.imshow("Depth Viewer", depth_image)
            key = cv2.waitKey(1)
            
            if key == ord('q') or key == ESC_KEY:
                break
            if key == ord('s'):

                # bot = Bot()
                # rxyz=bot.get_coord()
                # print("machine_pos=",rxyz)  
                # print(" ",rxyz,file=data1)
                # print(",\n")



                save_color_frame(color_frame, temp)
                print("color image saved")
                save_depth_frame(depth_frame, temp)
                print("depth image saved")                     

                temp+=1
                
        except KeyboardInterrupt:
            break
    pipeline.stop()
    data1.close()


if __name__ == "__main__":
    main()


	


    
