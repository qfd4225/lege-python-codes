###########################################################################################
# Copyright (c) OpenMMLab. All rights reserved.
import sys
sys.path.append('/home/lg/cjt_project/agv/mmdet/mmdeploy/build/lib')

import argparse
import os

import math
from PIL import Image
import time
import cv2
from mmdeploy_runtime import Detector
import numpy as np
import open3d as o3d
from sklearn.linear_model import LinearRegression

from pymycobot import ElephantRobot


def log(data_name, data):
    print("-----------LOG:\n", data_name, "=", data, "\n------------------")

class camera_mini:
    def __init__(self):
        # 定义相机内参
        self.cx = 317.883
        self.cy = 248.974
        self.fx = 515.395
        self.fy = 515.395
        self.RT = np.load("./test_pic/RT_camera_to_base.npy")
        log("RT", self.RT)


class camera_wide():
    def __init__(self):
        self.cx = 647.965271
        self.cy = 477.223999
        self.fx = 720.908447
        self.fy = 720.908447
        self.RT = np.load("./test_pic/RT_camera_to_base.npy")
        log("RT", self.RT)

# 抓取点类
class point_catch_mini:
    def __init__(self):
        camera = camera_mini()

        self.RT = camera.RT
        self.cx = camera.cx
        self.cy = camera.cy
        self.fx = camera.fx
        self.fy = camera.fy

        # 图片路径
        color_pic_path = './test_pic/c_t.png'
        deep_pic_path = "./test_pic/d_t.png"

        # 获取pic->np

        image_deep_temp = Image.open(deep_pic_path)
        self.image_deep = np.array(image_deep_temp)  # 读深度方式位 y, x
        self.image_color = cv2.imread(color_pic_path)
        image_deep_temp.close()

        self.mmdet_catch_point()

    # 通过图片计算世界坐标 x, y
    def word_point_cal_pic(self, p_x, p_y):
        # print("x = ", p_x)
        # print("y = ", p_y)
        deep = self.image_deep[p_y][p_x]
        X = (p_x - self.cx) * deep / self.fx
        Y = (p_y - self.cy) * deep / self.fy
        point_3d = [X, Y, deep]
        return point_3d
    
    def camera2base(self, point):
        A_homogeneous = np.append(point, 1)
        B_homogeneous = np.matmul(self.RT, A_homogeneous)
        B_cartesian = B_homogeneous[:-1] / B_homogeneous[-1]  # 笛卡尔坐标点B
        return B_cartesian.squeeze()

    # 使用mmdet进行目标检测分割
    def mmdet_catch_point(self):
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
            if index != 3:
                continue
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
                        if img_deep[y_step, x_step] >= 2500:
                            mask[y_step - top][x_step - left] = 0
                        if mask[y_step - top][x_step - left] == 255:
                            # 深度信息为0点pass
                            if img_deep[y_step, x_step] == 0:
                                continue
                            # word point cal
                            point_word = self.word_point_cal_pic(x_step, y_step)
                            # camera2base
                            point_base = self.camera2base(point_word)
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

                x_temp_ave = 0
                y_temp_ave = 0
                num_temp_ave = 0
                normal_rat = 0.93  # 垂直法向量的误差
                for i in range(len(points_catch)):
                    if abs(normals[i, 2]) > normal_rat:
                        x_temp_ave += points_catch[i, 0]
                        y_temp_ave += points_catch[i, 1]
                        num_temp_ave += 1
                        red[points_catch[i, 1]][points_catch[i, 0]] = 255
                        green[points_catch[i, 1]][points_catch[i, 0]] = 255
                        blue[points_catch[i, 1]][points_catch[i, 0]] = 255
                
                # get 2d catch points in pic
                x_ave = int(x_temp_ave / (num_temp_ave))
                y_ave = int(y_temp_ave / (num_temp_ave))

                # 2d 2 3d and camera2base
                point_cam = self.word_point_cal_pic(x_ave, y_ave)
                point_3d = self.camera2base(point_cam)
                log("point_3d", point_3d)

                # save 2d and 3d point
                catch_points_2d.append([x_ave, y_ave])
                catch_points_3d.append(point_3d)
                # log("catch_points_3d", catch_points_3d)

                # mask merge
                x0 = int(max(math.floor(bbox[0]) - 1, 0))
                y0 = int(max(math.floor(bbox[1]) - 1, 0))
                mask_img = blue[y0:y0 + mask.shape[0], x0:x0 + mask.shape[1]]
                cv2.bitwise_or(mask, mask_img, mask_img)    
                img_color = cv2.merge([blue, green, red])

                # draw Central point
                cv2.circle(img_color, (x_ave, y_ave), 3, (0, 0, 0), -1)
                # cv2.line(img, (x_0, int(y_0)), (x_1, int(y_1)), (0, 255, 255), 1) 

        cv2.imwrite('./test_pic/output_detection_w.png', img_color)
        cv2.imshow('output_detection.png', img_color)

        self.catch_points_2d = np.array(catch_points_2d)
        self.catch_points_3d = np.array(catch_points_3d)
        log("catch_points_2d", self.catch_points_2d)
        log("catch_points_3d", self.catch_points_3d)


class Bot:
    def __init__(self):
        self.elephant_client = ElephantRobot("10.17.235.83", 5001)
        self.elephant_client.start_client()

    def bot_run_coords(self, x, y, z, rx, ry, rz):
        # 运动到笛卡尔坐标
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


if __name__ == '__main__':
    catch_point = point_catch_mini()

    # 抓第一个箱子
    x = catch_point.catch_points_3d[0, 0]
    y = catch_point.catch_points_3d[0, 1]
    z = catch_point.catch_points_3d[0, 2] + 100

    # 抓取过程
    print("press R to run bot, Q to exit!")
    key = cv2.waitKey(0)
    if key == ord('q'):
        print("------------------over-----------------")
    elif key == ord('r'):
        print("------------------bot run-----------------")
        bot = Bot()
        # run to safe point
        # bot.bot_run_angles(9.780, -98.945, -81.864, -87.979, 88.506, 0.204)
        # run to up of catch point
        bot.bot_run_coords(x, y, z, 179, 1, -60)
        # run to catch point
        bot.bot_run_coord(2, z - 45)
        bot.bot_catch()
        # catch over up the bot
        bot.bot_run_coord(2, z + 50)
        # catch over
        bot.bot_catch_over()


