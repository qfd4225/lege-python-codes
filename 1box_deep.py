
# 图片路径
# './test_pic/c_t.png'
# "./test_pic/d_t.png"
import cv2
import math
from PIL import Image
import numpy as np
import open3d as o3d
from sklearn.linear_model import LinearRegression
import os

import sys
sys.path.append('/home/lg/cjt_project/agv/mmdet/mmdeploy/build/lib')

from mmdeploy_runtime import Detector

def log(data_name, data):
    print("-----------LOG:\n", data_name, "=", data, "\n------------------")

def show_img(img, img_name = "test"):
    cv2.imshow(img_name, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# 奇异值分解
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

class camera_mini:
    def __init__(self):
        # 定义相机内参
        self.cx = 317.883
        self.cy = 248.974
        self.fx = 515.395
        self.fy = 515.395
        k1 = 0.0517062
        k2 = -0.279072
        k3 = 0.28017
        p1 = -0.00100776
        p2 = 0.00304881
        self.dist = np.array([k1, k2, k3, p1, p2])

# 相机检测二维码类
class camera_code:
    def __init__(self):
        camera = camera_mini()

        self.cx = camera.cx
        self.cy = camera.cy
        self.fx = camera.fx
        self.fy = camera.fy
        self.dist = camera.dist

        # 二维码数量
        self.code_num = 4
        
        self.b_length = 449.0
        self.b_width = 351.0
        self.b_height = 217.0
        
        self.image_up = np.ones((int(self.b_width), int(self.b_length), 3), dtype=np.uint8) * 255


        # 图片路径
        color_pic_path = './test_pic/c_t.png'
        deep_pic_path = "./test_pic/d_t.png"
        
        color_pic_pnp = './test_pic/c1.png'

        # 标定板长宽
        self.row = 6
        self.col = 4

        # 定义矩形4个方向的极值
        self.up_edge = []
        self.down_edge = []
        self.right_edge = []
        self.left_edge = []

        # 获取pic->np
        image_deep_temp = Image.open(deep_pic_path)
        self.image_deep = np.array(image_deep_temp)  # 读深度方式位 y, x
        self.image_color = cv2.imread(color_pic_path)
        self.image_color_pnp = cv2.imread(color_pic_pnp)
        image_deep_temp.close()
        self.copy_c = self.image_color.copy()

        # 手动获取篮子角点坐标 计算pnp
        self.board_basket()
        log("points_basket", self.points_basket)
        self.basket = []
        self.get_pixel_coordinates(self.click_basket)
        log("basket", self.basket)
        if len(self.basket) < 7:
            self.basket = [[149.0, 98.0], [433.0, 112.0], [466.0, 312.0], [99.0, 303.0], [171.0, 185.0], [406.0, 193.0]]
        self.cal_Rt_pnp(self.points_basket, self.basket)

        # 获取标定板坐标和构建标定板坐标系 计算pnp或svd
        # self.corner_find_board()
        # self.board_system()
        # # # self.cal_Rt_svd()
        # log("points_board", self.points_board)
        # log("corners_2d", self.corners_2d)
        # self.cal_Rt_pnp(self.points_board, self.corners_2d)

        self.box_height()
        
        self.box_up_look = []
        
        self.mmdet_box_point()
        
        # log("log", self.box_up_look)

        # self.draw_box_height()

    # 通过图片计算世界坐标 x, y
    def word_point_cal_pic(self, p_x, p_y):
        deep = self.image_deep[p_y][p_x]
        X = (p_x - self.cx) * deep / self.fx
        Y = (p_y - self.cy) * deep / self.fy
        point_3d = [X, Y, deep]
        return point_3d

    # 查找二维码角点
    def corner_find_code(self):
        num_dete = self.code_num
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

        frame_c = self.image_color
        frame_d = self.image_deep
        gray = cv2.cvtColor(frame_c, cv2.COLOR_BGR2GRAY)
        color_copy = frame_c.copy()
        # 角点检测
        arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
        arucoParams = cv2.aruco.DetectorParameters()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)

        # 判断二维码个数
        if len(corners) == num_dete:
            # 获取角点的 id
            ids = ids.flatten()
            # ids排序
            sorted_indices = sorted(range(len(ids)), key=lambda k: ids[k])
            ids = [ids[i] for i in sorted_indices]
            corners = [corners[i] for i in sorted_indices]
            print("ids = ", ids)
            # print("corners = ", corners[0].squeeze())

            # 存点2d坐标 3d坐标 和 3d坐标为0的坐标
            corners_2d = []
            corners_3d = []
            e_points = []
            # 将corners 2d和3d点存储
            for i in range(len(corners)):
                for corner_temp in corners[i].squeeze():
                    # print('-----')
                    # print(corner_temp[0], corner_temp[1])
                    p_deep = frame_d[int(corner_temp[1])][int(corner_temp[0])]
                    if p_deep == 0:
                        e_points.append(i)
                    else:
                        p_word = self.word_point_cal_pic(int(corner_temp[0]), int(corner_temp[1]))
                        corners_3d.append(p_word)

                    corners_2d.append(corner_temp)

            self.corners_2d = np.array(corners_2d)
            self.corners_3d = np.array(corners_3d)
            self.e_points = np.array(e_points)

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
        show_img(color_copy)

    # 构建二维码坐标系建立
    def board_system_code(self):
        points_board = []
        # 二维码宽度
        code_w = 7
        # 棋盘格坐标 1 4 通用
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
        if self.code_num == 4:
            print("4 codes create")
            board_unm = 0
            for i in range(self.code_num * 4):
                e_temp = 0
                for e_i_temp in range(len(self.e_points)):
                    if i == self.e_points[e_i_temp]:
                        e_temp = 1
                if e_temp == 0:
                    points_board.append(points_board_temp[i])
                    board_unm += 1
            log("points_board", points_board)
            self.points_board = np.array(points_board)
        elif self.code_num == 1:
            print("1 codes create")
            board_unm = 0
            for i in range(self.code_num * 4):
                e_temp = 0
                for e_i_temp in range(len(self.e_points)):
                    if i == self.e_points[e_i_temp]:
                        e_temp = 1
                if e_temp == 0:
                    points_board.append(points_board_temp[i])
                    board_unm += 1
            log("points_board", points_board)
            self.points_board = np.array(points_board)
        else:
            print("create error")

    # 棋盘格角点
    # 查找棋盘格角点 返回角点坐标和角点世界坐标  横向角点、纵向角点、识别棋盘格数量、彩色图、深度图
    def corner_find_board(self, num_dete=1):
        frame_c = self.image_color
        frame_d = self.image_deep

        gray = cv2.cvtColor(frame_c, cv2.COLOR_BGR2GRAY)
        pattern_size = (self.row, self.col)
        self.e_points = []

        # 查找棋盘格  num_dete = 1 or 2 error_points为深度 = 0 点
        for i in range(num_dete):
            found, corners = cv2.findChessboardCorners(gray, pattern_size, None)
            if found:
                # 使用亚像素精确化方法提高角点检测的准确性
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

                # log("corners", corners)

                # 世界坐标检测
                self.corners_3d = []
                num = self.row * self.col
                for j in range(num):
                    point = corners[j]

                    # 获取corner点深度 计算世界坐标
                    p_deep = frame_d[int(point[0][1])][int(point[0][0])]
                    p_word = self.word_point_cal_pic(int(point[0][0]), int(point[0][1]))

                    if p_deep == 0:
                        self.e_points.append(j)
                    else:
                        self.corners_3d.append(p_word)

                # 绘制角点
                cv2.drawChessboardCorners(frame_c, pattern_size, corners, found)
                i = 0
                for corner in corners:
                    # cv2.circle(frame_c, (int(corner[0][0]), int(corner[0][1])), 5, (0, 0, 255 - i * 10), -1)
                    i += 1
                self.e_points = np.array(self.e_points)
                self.corners_3d = np.array(self.corners_3d)
                self.corners_2d = np.array(corners)
                show_img(frame_c)
            else:
                print("no board")
        log("e_points", self.e_points)
        log("corners_3d", self.corners_3d)

    # 构建棋盘格坐标系 棋盘格左上角作为原点
    def board_system(self):
        b_width = 30
        # 点数量
        self.points_board = np.zeros((self.row * self.col - len(self.e_points), 3))
        # 初始原点坐标
        start_boatd_word = np.array([0, 0, 0])
        temp_num = 0
        board_unm = 0
        for i in range(self.col):
            for j in range(self.row):
                e_temp = 0
                for e_i_temp in range(len(self.e_points)):
                    if temp_num == self.e_points[e_i_temp]:
                        e_temp = 1
                if e_temp == 0:
                    self.points_board[board_unm] = start_boatd_word + np.array([j * b_width, i * b_width, 0])
                    board_unm += 1
                temp_num += 1
        self.points_board = np.array(self.points_board)

    # 构建篮子坐标系
    def board_basket(self):
        # points_basket_temp = [[0, 0, 0],
        #                       [b_length, 0, 0],
        #                       [b_length, b_width, 0],
        #                       [0, b_width, 0]]
        points_basket_temp = [[0, 0, 0],
                              [self.b_length, 0, 0],
                              [self.b_length, self.b_width, 0],
                              [0, self.b_width, 0],
                              [0, 0, self.b_height],
                              [self.b_length, 0, self.b_height]]
        self.points_basket = np.array(points_basket_temp)

    # cal Rt A2B
    def cal_Rt_svd(self):
        R_cam2code, T_cam2code = rigid_transform_3D(self.corners_3d, self.points_board)
        self.RT_cam2code = np.zeros((4, 4), np.float64)
        self.RT_cam2code[:3, :3] = R_cam2code
        self.RT_cam2code[:3, 3] = np.array(T_cam2code).flatten()
        self.RT_cam2code[3, 3] = 1

    def cal_Rt_pnp(self, points_3d, points_2d):
        camera_matrix = np.array([[self.fx, 0., self.cx],
                                  [0., self.fy, self.cy],
                                  [0., 0., 1.]])
        # PnP运算
        (success, rotation_vector, translation_vector) = cv2.solvePnP(np.array(points_3d), np.array(points_2d), camera_matrix,
                                                                      self.dist,
                                                                      flags=cv2.SOLVEPNP_ITERATIVE)
        T_cam2code = translation_vector
        R_cam2code, _ = cv2.Rodrigues(rotation_vector)
        self.RT_cam2code = np.zeros((4, 4), np.float64)
        self.RT_cam2code[:3, :3] = R_cam2code
        self.RT_cam2code[:3, 3] = np.array(T_cam2code).flatten()
        self.RT_cam2code[3, 3] = 1

        cameraPoints = np.array([[-195.02204523, -169.74376158, 689.], [-165.85195821, -165.9737871, 690.], [-136.39882032, -161.95745011, 690.]], dtype=np.float32)
        cameraPoints = cameraPoints - translation_vector.T  # 平移
        cameraPoints = np.matmul(cameraPoints, cv2.Rodrigues(rotation_vector)[0])  # 旋转
        print("cameraPoints：", cameraPoints)

        # imagePoints_reprojected, _ = cv2.projectPoints(points_3d, rotation_vector, translation_vector, camera_matrix, self.dist)
        # reprojection_error = np.mean(np.linalg.norm(points_2d - imagePoints_reprojected, axis=1))
        # print("反投影误差：", reprojection_error)

        self.RT_cam2code = np.linalg.inv(self.RT_cam2code)


    # 利用Rt进行点坐标矩阵变换
    def Rt_Change(self, point_src):
        A_homogeneous = np.append(point_src, 1)
        B_homogeneous = np.matmul(self.RT_cam2code, A_homogeneous)
        point_dst = B_homogeneous[:-1] / B_homogeneous[-1]  # 笛卡尔坐标点B
        return point_dst

    # 单机获取篮子6个坐标（顺时针打标）
    def click_basket(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(self.basket) < 6:
            self.basket.append([x * 1.0, y * 1.0])
            cv2.circle(self.image_color_pnp, (x, y), 1, (255, 0, 0), -1)
            log("basket", self.basket)
        cv2.imshow('image', self.image_color_pnp)

    # 单击获取坐标
    def click_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.image_deep[y][x] != 0:
                cv2.circle(self.copy_c, (x, y), 1, (0, 0, 255), -1)
                camera_3d = self.word_point_cal_pic(x, y)
                code_3d = self.Rt_Change(camera_3d)
                log("camera_3d", camera_3d)
                log("code_3d", code_3d)
                # log("box_height", self.result_array[y, x])
            cv2.imshow('image', self.copy_c)

    def get_pixel_coordinates(self, event_name):
        cv2.imshow('image', self.image_color_pnp)
        cv2.setMouseCallback('image', event_name)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # 获取标定板坐标系下的箱子高度
    def box_height(self):
        frame_c = self.image_color
        frame_d = self.image_deep
        height, width = frame_d.shape
        print("test = ", height, width)

        # 定义新深度表
        self.result_array = np.empty((height, width), dtype=np.float32)

        # 对每个像素点进行遍历
        for y in range(height):
            for x in range(width):
                if frame_d[y][x] == 0:
                    category = -1
                else:
                    point_3d_temp = self.word_point_cal_pic(x, y)
                    point_3d_temp = self.Rt_Change(point_3d_temp)
                    category = point_3d_temp[2]

                self.result_array[y, x] = category

        print("self.result_array", self.result_array)

    # 使用mmdet进行篮子内箱子朝上面检测
    def mmdet_box_point(self):
        # save catch points
        catch_points_2d = []
        catch_points_3d = []

        img_color = self.image_color
        img_deep = self.image_deep

        # 创建detector 以及检测
        detector = Detector(model_path='./../mask_rcnn_model', device_name='cuda', device_id=0)
        bboxes, labels, masks = detector(img_color)

        indices = [i for i in range(len(bboxes))]
        # 分割
        for index, bbox, label_id in zip(indices, bboxes, labels):
            #if index != 0:
            #    continue
            [left, top, right, bottom], score = bbox[0:4].astype(int), bbox[4]
            # 评分>0.5作为箱子
            if score < 0.5:
                continue

            print('num============================ ', index)
            print('left = ', left)
            print('top = ', top)
            print('right = ', right)
            print('bottom = ', bottom)
            
            if masks[index].size:
                mask = masks[index]
                blue, green, red = cv2.split(img_color)
                mask_np = np.array(masks[index])

                # 3d point to cal vetor
                pcd = o3d.geometry.PointCloud()
                points = np.empty((0, 3), dtype=np.float32)
                
                # create net catch
                points_catch = []
                nat_temp = 3  # 网格密度
                for x_step in range(left, right, nat_temp):
                    for y_step in range(bottom, top, -nat_temp):
                        # too far point delete
                        #if img_deep[y_step, x_step] >= 2500:
                        #    mask[y_step - top][x_step - left] = 0
                        if mask[y_step - top][x_step - left] == 255:
                            # 深度信息为0点pass
                            if img_deep[y_step, x_step] == 0:
                                continue
                            # word point cal
                            point_word = self.word_point_cal_pic(x_step, y_step)
                            # Rt_Change
                            point_base = self.Rt_Change(point_word)
                            # save word point
                            points = np.vstack((points, point_base))
                            points_catch.append([x_step, y_step])
                        
                # save to pcd and cal vector
                pcd.points = o3d.utility.Vector3dVector(points)
                # cal vector
                pcd.estimate_normals()
                
                # vector 2 numpy
                normals = np.asarray(pcd.normals)
                points_catch = np.asarray(points_catch)
                # np.savetxt('normals.txt', normals)
                # np.savetxt('points_catch.txt', points_catch)

                # 左右最大值 最小值判断
                x_max = np.max(points[:, 0])
                x_min = np.min(points[:, 0])
                self.left_edge.append(x_min)
                self.right_edge.append(x_max)

                # 上下最大值最小值判断
                y_max = np.max(points[:, 1])
                y_min = np.min(points[:, 1])
                self.up_edge.append(y_min)
                self.down_edge.append(y_max)

                x_temp_ave = 0
                y_temp_ave = 0
                num_temp_ave = 0
                normal_rat = 0.8  # 垂直法向量的误差
                box_up_temp = []

                # log("points", points[1])
                # log("points", points[1][0])
                for i in range(len(points_catch)):
                    if points[i][0] < 0 or points[i][1] < 0 or points[i][0] > self.b_length or points[i][1] > self.b_width:
                        continue
                    if abs(normals[i, 2]) > normal_rat:
                    # if True:
                        box_up_temp.append(points[i])
                        self.box_up_look.append(points[i])
                        x_temp_ave += points_catch[i, 0]
                        y_temp_ave += points_catch[i, 1]
                        num_temp_ave += 1
                        red[points_catch[i, 1]][points_catch[i, 0]] = 255
                        green[points_catch[i, 1]][points_catch[i, 0]] = 255
                        blue[points_catch[i, 1]][points_catch[i, 0]] = 255

                # mask merge
                x0 = int(max(math.floor(bbox[0]) - 1, 0))
                y0 = int(max(math.floor(bbox[1]) - 1, 0))
                mask_img = blue[y0:y0 + mask.shape[0], x0:x0 + mask.shape[1]]
                cv2.bitwise_or(mask, mask_img, mask_img)    
                img_color = cv2.merge([blue, green, red])
                box_up_look = np.array(box_up_temp)
                # np.save('box_up_look', self.box_up_look)
                
                self.draw_box_height(box_up_look)

        cv2.imwrite('./test_pic/output_detection_w.png', img_color)
        show_img(img_color)
        # cv2.imshow('output_detection.png', self.copy_c)

        log("up", self.up_edge)
        log("down", self.down_edge)
        log("right", self.right_edge)
        log("left", self.left_edge)

        

        # self.draw_box_height(self.box_up_look)
    # 绘制俯视矩阵
    def draw_box_height(self, up_look):
        x_max = int(np.max(up_look[:,0]))
        x_min = int(np.min(up_look[:,0]))

        y_max = int(np.max(up_look[:,1]))
        y_min = int(np.min(up_look[:,1]))

        z_ave = np.mean(up_look[2])


        # 设置矩形的颜色（蓝色）
        rat = z_ave / self.b_height
        color = (255 * rat, 0, 0)  # 蓝色

        cv2.rectangle(self.image_up, (x_min, y_min), (x_max, y_max), color, -1)

        cv2.putText(self.image_up, str(int(z_ave)), (int((x_max + x_min)/2), int((y_max + y_min)/2)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        show_img(self.image_up)


if __name__ == '__main__':
    camera = camera_code()
    # log("e_points", camera.e_points)
    # log("corners_3d", camera.corners_3d)
    # log("points_board", camera.points_board)
    log("RT_cam2code", camera.RT_cam2code)

    camera.get_pixel_coordinates(camera.click_event)
