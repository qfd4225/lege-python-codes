import numpy as np
import cv2
import math
from PIL import Image


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from math import atan2, asin, degrees,sin,cos
import sys
sys.path.append('/home/lg/cjt_project/agv/mmdet/mmdeploy/build/lib')

import argparse
import os

import math
from PIL import Image
import time
import cv2
#from mmdeploy_runtime import Detector
import numpy as np
import open3d as o3d
from sklearn.linear_model import LinearRegression

from pymycobot import ElephantRobot

pi=3.14159265358979323846


def eulerAnglesToRotationMatrix1(theta):
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]])

    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]])

    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]])
    R = np.dot(R_x, np.dot(R_y, R_z))
    return R


class Bot:
    def __init__(self):
        self.elephant_client = ElephantRobot("10.17.235.83", 5001)
        self.elephant_client.start_client()

    def bot_run_coords(self, x, y, z, rx, ry, rz):
        
        self.elephant_client.write_coords([x, y, z, rx, ry, rz], 1500)
        self.elephant_client.command_wait_done()
        self.elephant_client.wait(1)

    def bot_run_angles(self, j0, j1, j2,j3, j4, j5):
        self.elephant_client.write_angles([j0, j1, j2,j3 ,j4 ,j5], 1000)
        self.elephant_client.command_wait_done()
        self.elephant_client.wait(1)
        
    def bot_catch(self):
        self.elephant_client.set_digital_out(0,1)
        self.elephant_client.wait(5)

    def bot_catch_over(self):
        self.elephant_client.set_digital_out(0,0)
        self.elephant_client.wait(1)
    
    def bot_run_coord(self, xyz, coord):
        self.elephant_client.write_coord(xyz, coord, 1500)
        self.elephant_client.command_wait_done()
        self.elephant_client.wait(1)
        
    def get_coord(self):
        xyz=self.elephant_client.get_coords()
        self.elephant_client.command_wait_done()
        self.elephant_client.wait(1)
        return xyz





mmpi=3.14159265358979323846


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

org_img_W = 640
org_img_H = 480

coordinates = []

i = 1
temp = str(i)
image_path_d = r"./test_pic/d_t.png"
image_path_c = r"./test_pic/c_t.png"
image_deep = Image.open(image_path_d)
image_deep_np = np.array(image_deep)


RT_camera_to_tool=np.array([[-0.9360792099421864, -0.310489171115365, -0.1653849670742389, 113.7453028033813],
 [-0.306952669636022, 0.9505540695768269, -0.04719130655460546, 2.096056184608122],
 [0.1718597431552311, 0.006590556205326004, -0.9850993824236838, 127.1405710380867],
 [0, 0, 0, 1]])


#RT_camera_to_tool= np.load("./test_pic/saveRT.npy")
# 法向量转欧拉角 # 输入法向量
def V_to_oura(normal):
    # 将法向量规范化为单位向量
    normal_unit = normal / np.linalg.norm(normal)
    # 定义参考向量
    reference_vector = np.array([0, 0, -1])

    # 计算旋转轴
    rotation_axis = np.cross(reference_vector, normal_unit)
    # 计算旋转角度
    rotation_angle = degrees(atan2(np.linalg.norm(rotation_axis), np.dot(reference_vector, normal_unit)))

    # 计算旋转矩阵
    rotation_matrix = np.array([[0, -rotation_axis[2], rotation_axis[1]],
                                [rotation_axis[2], 0, -rotation_axis[0]],
                                [-rotation_axis[1], rotation_axis[0], 0]])

    # 计算欧拉角
    euler_angles_rad = np.dot(rotation_matrix, normal_unit)
    euler_angles_deg = np.degrees(euler_angles_rad)
    return euler_angles_deg

# 深度图中世界坐标求解 横坐标、纵坐标、deep_pic
def word_point_cal_pic(p_x, p_y, deep_pic):
    # camera data
    print("x = ", p_x)
    print("y = ", p_y)
    deep = deep_pic[p_y][p_x]
    X = (p_x - cx_d) * deep / fx_d  # cx = self.camera_matrix
    Y = (p_y - cy_d) * deep / fy_d
    wd_p = [X, Y, deep]
    return wd_p

def undistort_point(input_point_X,input_point_Y):
    # 创建相机矩阵和畸变系数数组
    camera_matrix = np.array([[fx_d, 0, cx_d],
                               [0, fy_d, cy_d],
                               [0, 0, 1]], dtype=np.float64)
    
    dist_coeffs = np.array([k1, k2, p1, p2, k3, k4, k5, k6], dtype=np.float64)

    # 进行点的畸变校正
    distorted_point = np.array([[input_point_X, input_point_Y]], dtype=np.float64)
    undistorted_point = cv2.undistortPoints(distorted_point, camera_matrix, dist_coeffs, None, camera_matrix)

    # 提取校正后的点坐标
    output_point_X, output_point_Y = undistorted_point[0][0]


    return output_point_X, output_point_Y
def cal3d(p_x, p_y,p_x_undist,p_y_undist,deep_pic):
    print("x = ", p_x)
    print("y = ", p_y)
    deep = deep_pic[int(p_y)][int(p_x)]
    X = (p_x_undist - cx_d) * deep / fx_d  # cx = self.camera_matrix
    Y = (p_y_undist - cy_d) * deep / fy_d
    wd_p = [X, Y, deep]
    return wd_p
def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        if image_deep_np[y][x] != 0:
            
            cv2.circle(img, (x, y), 1, (0, 0, 255), -1)

            x1_tmp,y1_tmp=undistort_point(x, y)
            p_word = cal3d(x, y,x1_tmp,y1_tmp,image_deep_np)
            coordinates.append(p_word)

            print("(", word_point_cal_pic(x, y, image_deep_np), ")")
        cv2.imshow('image', img)

def get_pixel_coordinates(image_path):
    global img
    img = cv2.imread(image_path)
    cv2.imshow('image', img)
    cv2.setMouseCallback('image', click_event)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# 欧拉角转法向量：
# 欧拉角转选择矩阵
# 欧拉角转选择矩阵
def euler_to_rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    # 将角度转换为弧度
    roll = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_deg)
    yaw = np.deg2rad(yaw_deg)
    # 计算旋转矩阵
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    R = np.dot(Rz, np.dot(Ry, Rx))
    return R



def compute_normal_vector(pointA, pointB, pointC):
    """
    计算面的法向量

    参数：
    - pointA, pointB, pointC：三个点的坐标

    返回：
    - normal_vector：面的法向量
    """

    # 计算向量AB和AC
    AB = pointB - pointA
    AC = pointC - pointA

    # 计算法向量
    normal_vector = np.cross(AB, AC)

    return normal_vector

def euclidean_distance(point1, point2):
    x1, y1, z1 = point1
    x2, y2, z2 = point2
    distance = math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)
    return distance



def calculate_transform(x, y, z, rx, ry, rz):
    # 角度转弧度
    rx_rad = np.deg2rad(rx)
    ry_rad = np.deg2rad(ry)
    rz_rad = np.deg2rad(rz)

    # 创建旋转矩阵 R_tool2base
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx_rad), -np.sin(rx_rad)],
                   [0, np.sin(rx_rad), np.cos(rx_rad)]])

    Ry = np.array([[np.cos(ry_rad), 0, np.sin(ry_rad)],
                   [0, 1, 0],
                   [-np.sin(ry_rad), 0, np.cos(ry_rad)]])

    Rz = np.array([[np.cos(rz_rad), -np.sin(rz_rad), 0],
                   [np.sin(rz_rad), np.cos(rz_rad), 0],
                   [0, 0, 1]])

    R_tool2base =  np.dot(Rz, np.dot(Ry, Rx))

    # 创建平移矩阵 T_tool2base
    T_tool2base = np.array([[x],
                            [y],
                            [z]])

    # 创建变换矩阵 RT_tool2base
    RT_tool2base = np.hstack((R_tool2base, T_tool2base))
    RT_tool2base = np.vstack((RT_tool2base, np.array([0, 0, 0, 1])))

    return RT_tool2base, R_tool2base, T_tool2base

#用于根据欧拉角计算旋转矩阵
def myRPY2R_robot(x, y, z):
    Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
    Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
    Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
    R = Rz@Ry@Rx
    return R

#用于根据位姿计算变换矩阵
def pose_robot(x, y, z, Tx, Ty, Tz):

    thetaX = x / 180 * pi
    thetaY = y / 180 * pi
    thetaZ = z / 180 * pi
    R = myRPY2R_robot(thetaX, thetaY, thetaZ)
    t = np.array([[Tx], [Ty], [Tz]])
    RT1 = np.column_stack([R, t])  # 列合并
    RT1 = np.row_stack((RT1, np.array([0,0,0,1])))
    # RT1=np.linalg.inv(RT1)
    return RT1


def main():	
    get_pixel_coordinates(image_path_c)
    points = np.array(coordinates)
    print("points =",points)
    print("\nRT_camera_to_tool=",RT_camera_to_tool)

    bot = Bot()
    rxyz=bot.get_coord()
    print("machine pos=",rxyz)    


# 计算base->tool
    wT_temp = np.array(rxyz[:3])
    wT_temp = np.array(wT_temp).reshape([3, 1])

    woura_temp = np.array(rxyz[3:])
    wR_temp = euler_to_rotation_matrix(woura_temp[0], woura_temp[1], woura_temp[2])

    wRT_temp = np.zeros((4, 4), np.float64)
    wRT_temp[:3, :3] = wR_temp
    wRT_temp[:3, 3] = np.array(wT_temp).flatten()
    wRT_temp[3, 3] = 1

    # 求逆后保存 RT_data_inv
    wRT_data_inv_temp = np.linalg.inv(wRT_temp)

    # 获得RT的逆矩阵
    wR_data_inv_temp = wRT_data_inv_temp[:3, :3]
    wT_data_inv_temp = wRT_data_inv_temp[:3, 3]

    print("wR_data_inv_temp = ", wR_data_inv_temp)
    print("wT_data_inv_temp = ", wT_data_inv_temp)
    # t_cat = np.ones(1,1)

    wR_data_inv_temp = np.linalg.inv(wR_temp)
    wT_data_inv_temp = -np.matmul(wR_data_inv_temp, wT_temp)

    # RT放入
    wRT_data_inv_temp = np.zeros((4, 4), np.float64)
    wRT_data_inv_temp[:3, :3] = wR_data_inv_temp
    wRT_data_inv_temp[:3, 3] = np.array(wT_data_inv_temp).flatten()
    wRT_data_inv_temp[3, 3] = 1
    
    wRT_data_inv_temp11 = np.linalg.inv(wRT_data_inv_temp)


    print("RT_Tool2base=",wRT_data_inv_temp11)
    ay=[0,0,0]
    Base_cood = np.matmul(wRT_data_inv_temp11, np.append(ay, 1))
    print("Base_cood1=",rxyz[:3])
    print("Base_cood2=",Base_cood)
    
    RT_camera_to_base=np.dot(wRT_data_inv_temp11,RT_camera_to_tool)
    print("RT_camera_to_base=",RT_camera_to_base)


    Bd=[]
    for i in range(len(points)):
        point = points[i]
        p1=[0,0,0]
        A_homogeneous = np.append(p1, 1)
        B_homogeneous = np.matmul(RT_camera_to_base, A_homogeneous)
        B_cartesian = B_homogeneous[:-1] / B_homogeneous[-1]  # 笛卡尔坐标点B
        print("点B的坐标:", B_cartesian)
        Bd.append(B_cartesian)
       # dis = euclidean_distance(B_cartesian, (0, 0, 0))
       # print("dis = ", dis)
    
    for i in range(1,len(points)):
        
        
        print("points distance_cam = ", euclidean_distance(points[i],points[i-1]))
        print("points distance_rob = ", euclidean_distance(Bd[i],Bd[i-1]))




if __name__ == "__main__":
    main()

