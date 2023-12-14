import numpy as np
import cv2
import math
from PIL import Image
import time
import numpy as np
from math import atan2, asin, degrees,cos,sin
import glob
 


chess_board_x_num = 11
chess_board_y_num=8
chess_board_len=30







pi=3.14159265358979323846


# # mini参数
# cx = 317.883
# cy = 248.974
# fx = 515.395
# fy = fx

# # # 同一套
# fx_d = fx
# fy_d = fx
# cx_d = cx
# cy_d = cy


# k1 = 0.0517062
# k2 = -0.279072
# k3 = 0.28017
# p1 = -0.00100776
# p2 = 0.00304881


# k4=0 
# k5=0
# k6=0    


#Depth Intrinisc
ffx=314.681
ffy=314.681
ccx=319.425 
ccy=240.161


# #广角参数
fx = 360.454 
fy = 360.454 
cx = 315.851 
cy = 238.612
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

# # #广角参数
# fx = 360.26314113
# fy = 360.2315993
# cx = 315.7547886
# cy = 239.01939704
# fx_d = fx
# fy_d = fy
# cx_d = cx
# cy_d = cy
# k1=0.31660437 
# k2=-0.37466116
# k3= 0.04325664
# k4=0 
# k5=0
# k6=0
# p1=0.00081818
# p2=-0.00079844


# #广角参数
# fx = 362.14129854
# fy = 362.01809685
# cx = 315.92377635
# cy = 238.12134645
# fx_d = fx
# fy_d = fy
# cx_d = cx
# cy_d = cy
# k1=0.28740516
# k2=-0.17079917
# k3=-0.36513165
# k4=0 
# k5=0
# k6=0
# p1=0.00033838
# p2=-0.0007984


camera_matrix = np.array([[fx, 0., cx],
                          [0., fy, cy],
                          [0., 0., 1.]])
# 畸变系数
dist = np.array([k1, k2, p1, p2, k3])


org_img_W = 640
org_img_H = 480



import cv2
import numpy as np
import glob
 
# 二维码格子宽度
code_w = 7

# 二维码类型
aruco_type = "DICT_5X5_100"
# 二维码字典
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


# 构建二维码坐标系建立
def board_system(num_data, e_points):
    print("e_points=",e_points)
    points_board = []
    if num_data == 4:
        points_board_temp = [[0, 0, 0],
                             [code_w * 7, 0, 0],
                             [code_w * 7, code_w * 7, 0],
                             [0, code_w * 7, 0],
                             [code_w * 8, 0, 0],
                             [code_w * 15, 0, 0],
                             [code_w * 15, code_w * 7, 0],
                             [code_w * 8, code_w * 7, 0],
                             [code_w * 8, code_w * 8, 0],
                             [code_w * 15, code_w * 8, 0],
                             [code_w * 15, code_w * 15, 0],
                             [code_w * 8, code_w * 15, 0],
                             [0, code_w * 8, 0],
                             [code_w * 7, code_w * 8, 0],
                             [code_w * 7, code_w * 15, 0],
                             [0, code_w * 15, 0]]
        print("4 codes create")
        board_unm = 0
        for i in range(num_data * 4):
            e_temp = 0
            for e_i_temp in range(len(e_points)):
                if i == e_points[e_i_temp]:
                    e_temp = 1
            if e_temp == 0:
                points_board.append(points_board_temp[i])
                board_unm += 1
        print("points_board = ", points_board)
        points_board = np.array(points_board)
    elif num_data == 1:
        print("1 codes create")
    else:
        print("create error")
    return points_board


def bilinear_interpolation(ppp, ppx, ppy):
    uy = ppx
    ux = ppy
    x1 = math.floor(ux)
    x2 = math.ceil(ux)
    y1 = math.floor(uy)
    y2 = math.ceil(uy)

    fq11 = ppp[x1][y1]
    fq21 = ppp[x2][y1]
    fq12 = ppp[x1][y2]
    fq22 = ppp[x2][y2]

    f1 = fq11 * (x2 - ux) * (y2 - uy)
    f2 = fq21 * (ux - x1) * (y2 - uy)
    f3 = fq12 * (x2 - ux) * (uy - y1)
    f4 = fq22 * (ux - x1) * (uy - y1)

    print("fq11=",fq11)
    print("fq21=",fq21)
    print("fq12=",fq12)
    print("fq22=",fq22)

    fff = f1 + f2 + f3 + f4
    return fff

# 查找二维码角点 返回角点坐标和角点世界坐标
def corner_find(frame_c, frame_d, num_dete=4):
    # 有效点大于4
    error_temp = 0

    gray = cv2.cvtColor(frame_c, cv2.COLOR_BGR2GRAY)
    color_copy = frame_c.copy()
    # cv2.imshow('gr', gray)
    # cv2.waitKey(0)
    # gray = frame_c
    # 角点检测
    arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
    arucoParams = cv2.aruco.DetectorParameters()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)
    
    if len(corners) == num_dete:
        # 获取角点的 id
        ids = ids.flatten()
        # ids排序
        sorted_indices = sorted(range(len(ids)), key=lambda k: ids[k])
        ids = [ids[i] for i in sorted_indices]
        corners = [corners[i] for i in sorted_indices]
        print("ids = ", ids)
        print("corners = ", corners[0].squeeze())

        # 存点2d坐标 3d坐标 和 3d坐标为0的坐标
        corners_2d = []
        corners_3d = []
        e_points = []
        # 将corners 2d和3d点存储
        mum_rmp=0
        for i in range(len(corners)):
            for corner_temp in corners[i].squeeze():
                # print('-----')
                # print(corner_temp[0], corner_temp[1])
                p_deep = frame_d[int(corner_temp[1])][int(corner_temp[0])]
                if p_deep == 0:
                    e_points.append(mum_rmp)
                else:
                    p_word = word_point_cal(int(corner_temp[0]), int(corner_temp[1]), p_deep)
                    corners_3d.append(p_word)
                    corners_2d.append(corner_temp)
                mum_rmp+=1

        corners_2d = np.array(corners_2d)
        corners_3d = np.array(corners_3d)
        e_points = np.array(e_points)

        print("corners_2d = ", corners_2d)
        print("corners_3d = ", corners_3d)
        print("e_points = ", e_points)

        if len(corners_3d) <= 4:
            print("right points num = ", len(corners_3d))
            error_temp = 1

    # 可视化
    point_id_ = 0
    for (markerCorner, markerID) in zip(corners, ids):
        # 左上，右上，右下，左下 由二维码的形状决定
        corners_ = markerCorner.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners_
        # 确定坐标
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))
        # 角点深度检测，深度为 0 不采纳
        # 画框
        cv2.line(color_copy, topLeft, topRight, (0, 255, 0), 2)
        cv2.line(color_copy, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(color_copy, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(color_copy, bottomLeft, topLeft, (0, 255, 0), 2)
        cv2.putText(color_copy, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0), 2)
        for corner_pt in markerCorner.squeeze():
            corner_pt = corner_pt.squeeze()
            cv2.putText(color_copy, str(point_id_), (int(corner_pt[0]), int(corner_pt[1])),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 0, 0), 1)
            point_id_ += 1
    cv2.imshow("Chessboard Detection-color_copy", color_copy)    
    cv2.waitKey(0)

    return error_temp, corners_2d, corners_3d, e_points



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

# 机械臂末端坐标以及欧拉角：  广角


bot_end_data = [
  [-15.892161, -294.845779, 541.923475, 169.133774, 7.108664, -95.406256],
  [-16.022489, -295.125553, 542.097955, 171.103995, 7.942442, -86.385574],
  [-15.809602, -295.34774, 542.503146, 169.709583, 7.944014, -77.601867],
  [-15.961382, -295.251778, 542.490059, 169.575447, 4.154117, -65.275188],
  [-15.665775, -294.623833, 541.82899, 171.79697, 13.309939, -46.884158],
  [-16.304433, -294.626487, 541.611405, 168.683402, 19.610028, -31.103789],
  [-15.359613, -294.433697, 541.628106, 170.36364, 12.196775, -12.127508],
  [-15.818651, -295.419903, 542.665376, 165.114364, 16.09886, -42.790872],
  [-16.128853, -294.535441, 541.713881, 163.065147, 21.033652, -59.224134],
  [-15.164387, -295.136017, 542.316228, 157.957114, 15.06833, -69.834532],
  [-15.291619, -295.172702, 542.448985, 167.61105, 6.824828, -81.24262],
  [-15.948222, -295.140143, 542.204821, 162.884173, 0.695654, -104.182024],
  [-15.92613, -295.300622, 542.571748, 169.191737, -7.034771, -116.159344],
  [-15.817646, -295.411383, 542.511403, 165.791882, -9.399337, -129.29025],
  [-15.292883, -294.833957, 542.183051, 174.597505, 4.778294, -89.586525],
  [-16.373831, -294.347793, 541.182711, 171.277684, 15.397483, -26.170173]
]

# 坐标组数 对应图片数量 循环数量
l_bot = len(bot_end_data)
num_temp = l_bot




# 深度相机下的世界坐标求解 横坐标、纵坐标、点深度信息
def word_point_cal(p_x, p_y, deep):
    # camera data
    X = (p_x - cx_d) * deep / fx_d  # cx = self.camera_matrix
    Y = (p_y - cy_d) * deep / fy_d
    wd_p = [X, Y, deep]
    return wd_p



def cal3d(p_x, p_y,p_x_undist,p_y_undist,deep_pic):
    print("x = ", p_x)
    print("y = ", p_y)
    deep = deep_pic[int(p_y)][int(p_x)]
    X = (p_x_undist - cx_d) * deep / fx_d  # cx = self.camera_matrix
    Y = (p_y_undist - cy_d) * deep / fy_d
    wd_p = [X, Y, deep]
    return wd_p

def cal3d_q(p_x_undist,p_y_undist,deep):
    X = (p_x_undist - cx_d) * deep / fx_d  # cx = self.camera_matrix
    Y = (p_y_undist - cy_d) * deep / fy_d
    wd_p = [X, Y, deep]
    return wd_p

def cal3d1(p_x,p_y,deep):

    X = (p_x - cx_d) * deep / fx_d  # cx = self.camera_matrix
    Y = (p_y - cy_d) * deep / fy_d
    wd_p = [X, Y, deep]
    return wd_p
    

# 深度图中世界坐标求解 横坐标、纵坐标、deep_pic
def word_point_cal_pic(p_x, p_y, deep_pic):
    # camera data
    deep = deep_pic[p_y][p_x]
    X = (p_x - cx_d) * deep / fx_d  # cx = self.camera_matrix
    Y = (p_y - cy_d) * deep / fy_d
    wd_p = [X, Y, deep]
    return wd_p


def undistort_image(img):
    # 读取输入图像 

    # 设置相机矩阵和畸变系数
    camera_matrix = np.array([[fx_d, 0, cx_d],[0, fy_d, cy_d],[0, 0, 1]], dtype=np.float64)
    
    dist_coeffs = np.array([k1, k2, p1, p2, k3, k4, k5, k6], dtype=np.float64)

    # 进行相机畸变校正
    undistorted_img = cv2.undistort(img, camera_matrix, dist_coeffs)

    return undistorted_img


def get_caliboard_corners(imagecolor,rows,cols,imagedeepnp):
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

        corner_points = np.zeros((2, corners.shape[0]), dtype=np.float64)
        error_id=[]
        corners_3d = []
        corners_2d=[]
        corners_dist=[]
        for i in range(len(corners)):
            corner_points[:, i] = corners[i, 0, :]

            x, y = corners[i].ravel()
            x1,y1=undistort_point(x, y)
            cv2.circle(imagecolor, (int(x), int(y)), 2, (0, 255, 255), 1,cv2.LINE_8)     
            cv2.circle(imgdist, (int(x1), int(y1)), 2, (0, 255, 255), 1,cv2.LINE_8)     
            dxp=bilinear_interpolation(imagedeepnp,x,y)

            #dxp= imagedeepnp[int(y)][int(x)]
            if dxp==0:
                print("深度0错误!!!!")         
                error_id.append(i)
                
            p_word = word_point_cal((x),(y),dxp)

            corners_2d.append((x,y))
            corners_dist.append((x1,y1))

            font = cv2.FONT_HERSHEY_SCRIPT_SIMPLEX  
            textcolor = str(i)+":"#+str(imagedeepnp[int(y)][int(x)])
            textdist = str(i)+":"#+str(imagedeepnp[int(y1)][int(x1)])
            color = (0, 0, 255)
            thickness = 1
            font_scale = 0.3
            print("像素点:",str(i),",深度值:",str(dxp))
                
            cv2.putText(imagecolor, textcolor, (int(x), int(y)), font, font_scale, color, thickness)
            cv2.putText(imgdist, textdist, (int(x1), int(y1)), font, font_scale, color, thickness)
            corners_3d.append(p_word)
            
                        
        corners_3d=np.array( corners_3d)
        corners_2d=np.array( corners_2d)
        corners_dist=np.array(corners_dist)         
    else:
        print("无法找到角点")
    return corners_2d,corners_dist,corners_3d,imgdist,error_id

# 构建标定板坐标系建立
def chessboard_grid(row, col, length):
    points_board = []    
    for i in range(col):
        for j in range(row):
            x = i * length
            y = j * length
            z = 0  # Assuming a 2D chessboard, so set z to 0
            points_board.append([x, y, z])    
    points_board = np.array(points_board, dtype=np.float32)    
    return points_board



def calculate_transform_matrix(points_board, corners_3d):
    assert len(points_board) == len(corners_3d) and len(points_board) >= 3, "输入点的数量不匹配或太少"

    # 计算两组点的质心
    centroid_board = np.mean(points_board, axis=0)
    centroid_3d = np.mean(corners_3d, axis=0)

    # 居中化点集
    centered_board = points_board - centroid_board
    centered_3d = corners_3d - centroid_3d

    # 计算协方差矩阵
    H = np.dot(centered_board.T, centered_3d)

    # 使用奇异值分解 (SVD)
    U, _, Vt = np.linalg.svd(H)

    # 计算旋转矩阵 R
    R = np.dot(Vt.T, U.T)

    # 计算平移矩阵 T
    T = centroid_3d - np.dot(R, centroid_board)

    return R, T





def rigid_transform_3D(A, B):
    assert len(A) == len(B)
    # B = np.array(B)[...,None]
    # A = A[...,None]
    N = A.shape[0]  # total points
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # centre the points
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))

    H = np.matmul(np.transpose(AA), BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.matmul(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
        print("Reflection detected")
        Vt[2, :] *= -1
        R = np.matmul(Vt.T, U.T)

    t = -np.matmul(R, centroid_A) + centroid_B
    # err = B - np.matmul(A,R.T) - t.reshape([1, 3])
    return R, t

# 三维点之间距离计算
def euclidean_distance(point1, point2):
    x1, y1, z1 = point1
    x2, y2, z2 = point2
    distance = math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)
    return distance


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



# 法向量转欧拉角 # 输入法向量
def V_to_oura(normal):
    # 将法向量规范化为单位向量
    normal_unit = normal / np.linalg.norm(normal)
    # 定义参考向量
    reference_vector = np.array([0, 0, 1])

    # 计算旋转轴
    rotation_axis = np.cross(reference_vector, normal)
    # 计算旋转角度
    rotation_angle = degrees(atan2(np.linalg.norm(rotation_axis), np.dot(reference_vector, normal)))

    # 计算旋转矩阵
    rotation_matrix = np.array([[0, -rotation_axis[2], rotation_axis[1]],
                                [rotation_axis[2], 0, -rotation_axis[0]],
                                [-rotation_axis[1], rotation_axis[0], 0]])

    # 计算欧拉角
    euler_angles_rad = np.dot(rotation_matrix, normal)
    euler_angles_deg = np.degrees(euler_angles_rad)
    return euler_angles_rad, euler_angles_deg

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
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R




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


def compute_rigid_transform(point_set1, point_set2):
    """
    计算两组三维点系之间的刚体变换矩阵（旋转和平移）。

    参数：
    point_set1: 第一个点系，形状为 (N, 3) 的NumPy数组，N是点的数量。
    point_set2: 第二个点系，形状为 (N, 3) 的NumPy数组，N是点的数量。

    返回值：
    R: 3x3 的旋转矩阵
    t: 3x1 的平移向量
    """

    # 计算两个点系的质心
    centroid1 = np.mean(point_set1, axis=0)
    centroid2 = np.mean(point_set2, axis=0)

    # 将点系中心移到原点
    centered1 = point_set1 - centroid1
    centered2 = point_set2 - centroid2

    # 计算协方差矩阵
    H = np.dot(centered1.T, centered2)

    # 使用奇异值分解（SVD）计算旋转矩阵
    U, _, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # 处理反射变换问题
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = np.dot(Vt.T, U.T)

    # 计算平移矩阵
    t = centroid2.reshape(3, 1) - np.dot(R, centroid1.reshape(3, 1))

    return R, t





img_points1 = []  
obj_points1=[]
if __name__=="__main__":

        # 找棋盘格角点
    # 阈值
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # 世界坐标系中的棋盘格点,例如(0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)，去掉Z坐标，记为二维矩阵
    objp = np.zeros((chess_board_x_num*chess_board_y_num,3), np.float32)
    objp[:,:2] = np.mgrid[0:chess_board_x_num,0:chess_board_y_num].T.reshape(-1,2)
    # 储存棋盘格角点的世界坐标和图像坐标对
    objpoints = [] # 在世界坐标系中的三维点
    imgpoints = [] # 在图像平面的二维点

    images = glob.glob('./save_images_code/c*.png') # 标定所用图像
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # 找到棋盘格角点
        # 棋盘图像(8位灰度或彩色图像)  棋盘尺寸  存放角点的位置
        ret, corners = cv2.findChessboardCorners(gray, (chess_board_x_num,chess_board_y_num),None)
        # 如果找到足够点对，将其存储起来
        if ret == True:
            # 角点精确检测
            # 输入图像 角点初始坐标 搜索窗口为2*winsize+1 死区 求角点的迭代终止条件
            cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            objpoints.append(objp)
            imgpoints.append(corners)
            # 将角点在图像上显示
            cv2.drawChessboardCorners(img, (chess_board_x_num,chess_board_y_num), corners, ret)
            cv2.imshow('findCorners',img)
            cv2.waitKey(1000)
    cv2.destroyAllWindows()
    #标定、去畸变
    # 输入：世界坐标系里的位置 像素坐标 图像的像素尺寸大小 3*3矩阵，相机内参数矩阵 畸变矩阵
    # 输出：标定结果 相机的内参数矩阵 畸变系数 旋转矩阵 平移向量
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("mtx=",mtx)
    print("dist=",dist)
    


    R_tool2base_totol = []
    T_tool2base_totol = []
    RT_tool2base_totol = []   
    R_cal2cam_totol = []
    T_cal2cam_totol = []
    RT_cal2cam_totol = []


    # 标定图像

    code_num = 4

    for numw in range(0, num_temp):    
    
        temp = str(numw)
        image_path_d = r"./save_images_code/d"+ temp +".png"
        image_path_c = r"./save_images_code/c"+ temp +".png"
        image_deep = Image.open(image_path_d)
        image_deep_np = np.array(image_deep)

        image_color = cv2.imread(image_path_c)

#————————————————————————————————————————————————————————————————
    #     # 二维码标定板
        # error_temp, corners_2d, corners_3d, e_points = corner_find(image_color, image_deep_np, code_num)    
        
        # if error_temp == 1:
        #     print("pic ", numw, "not enough right points!!!!!!!!!!!!!!!!!!!!!")
        #     continue
        # print("enough right points")

        # points_board = board_system(code_num, e_points)


        # print("corners_2d.len=",len(corners_2d))
        # print("points_board.len=",len(points_board))
        # print("corners_3d.len = ", len(corners_3d))

        
        # if(len(corners_3d)==len(points_board)):
        #     for rf in range(0,len(corners_3d)):  

        #         if (rf%4!=0) :               
                    
        #             dist1=euclidean_distance(points_board[rf-1],points_board[rf]) 
        #             dist2=euclidean_distance(corners_3d[rf-1],corners_3d[rf])               

        #             print("dist_points_board=",dist1,"          dist_corners=",dist2)

        #             if(dist2>52.0 or dist2<46.0):
        #                 print("距离有问题！！！！")
        # else:
        #     print("角点不是16个!!!!")

        # 棋盘格标定板

        # 检测棋盘格角点坐标并3维重建
        corners_2d,corners_2d_dist,corners_3d,imagedist,error_id=get_caliboard_corners(image_color,chess_board_x_num,chess_board_y_num,image_deep_np)     
    
        points_board=chessboard_grid(chess_board_y_num,chess_board_x_num,chess_board_len)

        print("points_board.len()=",len(points_board))
        print("corners_2d_dist.len()=",len(corners_2d_dist))
        print("corners_2d.len()=",len(corners_2d))
        print("corners_3d.len()=",len(corners_3d))

        
        print("corners_2d=",corners_2d)
        print("points_board=",points_board)
        print("corners_3d = ", corners_3d)



        for i1 in range(chess_board_y_num*chess_board_x_num):
            for e_i in range(len(error_id)):                    
                if i1==error_id[e_i]:
                    points_board.pop(i1)
                    corners_2d_dist.pop(i1)
                    corners_2d.pop(i1)
                    corners_3d.pop(i1)



        print("points_board.len1()=",len(points_board))
        print("corners_2d_dist.len1()=",len(corners_2d_dist))
        print("corners_2d.len1()=",len(corners_2d))
        print("corners_3d.len1()=",len(corners_3d))



        font = cv2.FONT_HERSHEY_DUPLEX  
        color = (0, 0, 255)
        thickness = 1
        font_scale = 0.3

            

        cv2.namedWindow('Corners_normal', cv2.WINDOW_NORMAL)  # 设置窗口属性为可调整大小
        cv2.setWindowProperty('Corners_normal', cv2.WINDOW_NORMAL, cv2.WINDOW_KEEPRATIO)  # 保持宽高比
        cv2.namedWindow('Corners_dist', cv2.WINDOW_NORMAL)  # 设置窗口属性为可调整大小
        cv2.setWindowProperty('Corners_dist', cv2.WINDOW_NORMAL, cv2.WINDOW_KEEPRATIO)  # 保持宽高比
        cv2.imshow('Corners_normal', image_color)        
        cv2.imshow('Corners_dist', imagedist)       
        cv2.waitKey(0)
        # 关闭窗口
        cv2.destroyAllWindows()

#————————————————————————————————————————————————————————————————
        if(len(points_board)==len(corners_3d)==len(corners_2d)==len(corners_2d_dist)):
            print("lens::",len(points_board),len(corners_3d),len(corners_2d))
            dist1234214 = np.array([0, 0, 0, 0, 0])
            retval,rvec,tvec  = cv2.solvePnP(np.array(points_board, dtype=np.float64),np.array(corners_2d, dtype=np.float64), cv2.UMat(camera_matrix), cv2.UMat(dist))  
            rvec_matrix, _ = cv2.Rodrigues(rvec.get())
            print("R0 = ", rvec_matrix, "T0 = ", tvec.get())
            


        # 奇异值分解计算 cal->cam

#    #2D pnp
#             RT_temp0 = np.zeros((4, 4), np.float64)
#             RT_temp0[:3, :3] = rvec_matrix
#             RT_temp0[:3, 3] = np.array(tvec.get()).flatten()
#             RT_temp0[3, 3] = 1
#             np.set_printoptions(precision=8, suppress=True)
#             R_cal2cam_totol.append(rvec_matrix)
#             T_cal2cam_totol.append(tvec.get())
#             RT_cal2cam_totol.append(RT_temp0)
    #    # 3D pnp    
            R_cal2cam, T_cal2cam = rigid_transform_3D(points_board, corners_3d)
            print("R2 = ", R_cal2cam, "T2 = ", T_cal2cam)
            RT_temp0 = np.zeros((4, 4), np.float64)
            RT_temp0[:3, :3] = R_cal2cam
            RT_temp0[:3, 3] = np.array(T_cal2cam)
            RT_temp0[3, 3] = 1
            np.set_printoptions(precision=8, suppress=True)                                
            R_cal2cam_totol.append(R_cal2cam)
            T_cal2cam_totol.append(T_cal2cam.flatten())
            RT_cal2cam_totol.append(RT_temp0)


            for r1f in range(0,len(corners_3d)):        	
                points_board1=points_board[r1f]
                icamcoord1 = np.matmul(RT_temp0, np.append(points_board1, 1))
                #print("jiaodian_cal = ", icamcoord1, "jiaodian_stand = ", corners_3d[r1f])
                print("jiaodian_cal-jiaodian_stand = ", euclidean_distance(icamcoord1[0:3],corners_3d[r1f]))
                if(euclidean_distance(icamcoord1[0:3],corners_3d[r1f])>4):
                    print("距离太大了吧!组数:",numw)
                    break
            print("RT_temp0=",RT_temp0)
            

    # 计算base->tool
            wT_temp = np.array(bot_end_data[numw][:3])
            wT_temp = np.array(wT_temp).reshape([3, 1])

            woura_temp = np.array(bot_end_data[numw][3:])
            wR_temp = euler_to_rotation_matrix(woura_temp[0], woura_temp[1], woura_temp[2])
            print("aiai=",woura_temp[0], woura_temp[1], woura_temp[2],wR_temp)
            wRT_temp = np.zeros((4, 4), np.float64)
            wRT_temp[:3, :3] = wR_temp
            wRT_temp[:3, 3] = np.array(wT_temp).flatten()
            wRT_temp[3, 3] = 1

            # 求逆后保存 RT_data_inv
            wRT_data_inv_temp = np.linalg.inv(wRT_temp)

            # 获得RT的逆矩阵
            wR_data_inv_temp = wRT_data_inv_temp[:3, :3]
            wT_data_inv_temp = wRT_data_inv_temp[:3, 3]

            # print("wR_data_inv_temp = ", wR_data_inv_temp)
            # print("wT_data_inv_temp = ", wT_data_inv_temp)
            # t_cat = np.ones(1,1)

            wR_data_inv_temp = np.linalg.inv(wR_temp)
            wT_data_inv_temp = -np.matmul(wR_data_inv_temp, wT_temp)

            # RT放入
            wRT_data_inv_temp = np.zeros((4, 4), np.float64)
            wRT_data_inv_temp[:3, :3] = wR_data_inv_temp
            wRT_data_inv_temp[:3, 3] = np.array(wT_data_inv_temp).flatten()
            wRT_data_inv_temp[3, 3] = 1

            wRT_data_inv_temp11 = np.linalg.inv(wRT_data_inv_temp)
                    
            wR_data_inv_temp11 = wRT_data_inv_temp11[:3, :3]
            wT_data_inv_temp11 = wRT_data_inv_temp11[:3, 3]

            print("d0:",wRT_data_inv_temp11)
            print("d00:",wRT_temp)     
            
            print("d1:",wR_data_inv_temp11)
            print("d11:",wR_temp)
            print("d2:",wT_data_inv_temp11)
            print("d22:",wT_temp)




            ay=[0,0,0]
            Base_cood1 = np.matmul(wRT_data_inv_temp11, np.append(ay, 1))
            print("Base_cood_standard=",bot_end_data[numw][:3])
            print("Base_cood_calculate=",Base_cood1)
            
            
            
        

            # 计算tool->base1
            xx=bot_end_data[numw][0]
            yy=bot_end_data[numw][1]
            zz=bot_end_data[numw][2]
            rx=bot_end_data[numw][3]
            ry=bot_end_data[numw][4]
            rz=bot_end_data[numw][5]
            sRx = math.radians(rx)
            sRy = math.radians(ry)
            sRz = math.radians(rz)

            RT_temp, R_temp, T_temp=calculate_transform(xx,yy,zz,rx,ry,rz)

            # 需要将角度值转换为弧度值
            a = np.array([xx, yy, zz])
            stheta = [sRx, sRy, sRz]
            # 将转换结果放入m
            m = eulerAnglesToRotationMatrix1(stheta)
            # 合成旋转矩阵
            d1 = np.array([0, 0, 0, 1])
            b1 = np.row_stack((np.column_stack((m, a)), d1))
            

            print("RT=",RT_temp)


            print("真实值输出\nwRT_data_inv_temp11:",wRT_data_inv_temp11)
            print("RT_temp0:",RT_temp0)
            

            R_tool2base_totol.append(wR_data_inv_temp11)
            T_tool2base_totol.append(wT_data_inv_temp11)        
            RT_tool2base_totol.append(wRT_data_inv_temp11)




            print("len(R_tool2base_totol)=",len(R_tool2base_totol))
            print("len(T_tool2base_totol)=",len(T_tool2base_totol))
            print("len(R_cal2cam_totol)=",len(R_cal2cam_totol))
            print("len(T_cal2cam_totol)=",len(T_cal2cam_totol))



        print("len(R_tool2base_totol)=",len(R_tool2base_totol))
        print("len(T_tool2base_totol)=",len(T_tool2base_totol))
        print("len(R_cal2cam_totol)=",len(R_cal2cam_totol))
        print("len(T_cal2cam_totol)=",len(T_cal2cam_totol))
            
        


    R_camera_to_tool1, T_camera_to_tool1 = cv2.calibrateHandEye(R_tool2base_totol, T_tool2base_totol,R_cal2cam_totol, T_cal2cam_totol,method=cv2.CALIB_HAND_EYE_TSAI)     
   


    RT_camera_to_tool1 = np.zeros((4, 4), np.float64)
    RT_camera_to_tool1[:3, :3] = R_camera_to_tool1
    RT_camera_to_tool1[:3, 3] = np.array(T_camera_to_tool1).flatten()
    RT_camera_to_tool1[3, 3] = 1

    print("CALIB_HAND_EYE_TSAI = ", RT_camera_to_tool1)

    R_camera_to_tool2, T_camera_to_tool2 = cv2.calibrateHandEye(R_tool2base_totol, T_tool2base_totol,R_cal2cam_totol, T_cal2cam_totol,method=cv2.CALIB_HAND_EYE_PARK)     
   


    RT_camera_to_tool2 = np.zeros((4, 4), np.float64)
    RT_camera_to_tool2[:3, :3] = R_camera_to_tool2
    RT_camera_to_tool2[:3, 3] = np.array(T_camera_to_tool2).flatten()
    RT_camera_to_tool2[3, 3] = 1
    

    print("CALIB_HAND_EYE_PARK = ", RT_camera_to_tool2)

    print('version:',cv2.__version__)
    np.save('saveRT.npy',RT_camera_to_tool1)

    print("_________________________________________________结果1")
    for i in range(len(T_tool2base_totol)):
        chessPos=[ 0.0,0.0,0.0,1.0]
        camPos=np.matmul(RT_cal2cam_totol[i],chessPos)
        toolPos=np.matmul(RT_camera_to_tool1,camPos)
        worldPos=np.matmul(RT_tool2base_totol[i],toolPos)
        print("worldPos:",worldPos)

    print("_________________________________________________结果2")
    for i in range(len(T_tool2base_totol)):
        chessPos=[ 0.0,0.0,0.0,1.0]
        camPos=np.matmul(RT_cal2cam_totol[i],chessPos)
        toolPos=np.matmul(RT_camera_to_tool2,camPos)
        worldPos=np.matmul(RT_tool2base_totol[i],toolPos)
        print("worldPos:",worldPos)



   
