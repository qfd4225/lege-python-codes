import random
import time
import open3d as o3d
import numpy as np
import colorsys
import matplotlib.pyplot as plt
from distinctipy import distinctipy
from display_open3d import Ui_MainWindow
from PyQt5.QtCore import Qt
import pybullet as p
import pybullet_data
import sys
from PIL import Image, ImageDraw, ImageFont
from PyQt5.QtGui import (QColor,QPen, QPainter, QPainterPath, QPolygonF, QBrush,
                         QLinearGradient, QConicalGradient, QRadialGradient, QGradient,
                         QPixmap)

from PyQt5.QtWidgets import QApplication,QTableWidget,QWidget,QTableWidgetItem


from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import QTimer

from PyQt5.QtGui import QWindow
import win32gui
import cv2
# 置物篮尺寸
box_w = 12
box_l = 11
box_h = 13

centers=[]


#############计算稳定度

#节点,每个节点有两个后继节点，分别为左子节点和右子节点。
class Box_Node:
    def __init__(self, item,Lchild=None,Rchild=None):
        self.elem=item
        self.length = item[0]
        self.width = item[1]
        self.height = item[2]
        self.x = item[3]
        self.y = item[4]
        self.z = item[5]
        #父节点
        self.parent = None
        # 子节点
        self.lchild = Lchild
        self.rchild = Rchild
    def output(self):
        return (self.elem)

class Box_Tree:
    def __init__(self, items,Root=None,Busy_items=None,Lazy_items=None):
        self.elem = items
        self.root = Root
        self.busy_items = Busy_items
        self.lazy_items = Lazy_items
        self.box_matrix1 = []
        self.box_matrix = []

    def insert(self, element):
        node = Box_Node(element)
        if self.root == None:
            self.root = node
        else:
            queue = []
            queue.append(self.root)

            # 每插入一次都要找到树结构中最后一个叶子节点的位置
            while queue:
                cur = queue.pop(0)
                if cur.left == None:
                    cur.left = node
                    return
                elif cur.right == None:
                    cur.right = node
                    return
                else:
                    queue.append(cur.left)
                    queue.append(cur.right)
    def output_root(self):

        if self.root == None:
            return

        print(self.root)

    def output_elem(self):
        print(self.elem)

    #构造树 每个树有个根节点。

    def build_tree(self):
        boxes = []
        for info in self.elem:
            boxes.append(Box_Node(info))
        for i in range(len(boxes)):
            j_array=[]                              #i上边的东西
            for j in range(len(boxes)):
                if self.is_below(boxes[i], boxes[j]):#前边在后边的下边
                    #print(j," beyond ",i)
                    boxes[j].lchild=boxes[i]#上面的子节点是下边
                    j_array.append(j)
            #序号 上面接触的箱子序号


            if self.lazy_items != None and i in self.lazy_items:
                self.box_matrix1.append([i, []])
            else:
                self.box_matrix1.append([i, j_array])

        for i in range(len(boxes)):
            j_array=[]                              #i下边的东西
            for j in range(len(boxes)):
                if self.is_below(boxes[j], boxes[i]):#前边在后边的下边
                    #print(i," beyond ",j)
                    j_array.append(j)
            #序号 上面接触的箱子序号
            box_matrix_per = [self.box_matrix1[i][0],self.box_matrix1[i][1], j_array]
            self.box_matrix.append(box_matrix_per)


        for i in range(len(boxes)):
            if boxes[i].lchild == None:
                pass
                #print("没有子节点的箱子:",i)

        return boxes[0]
    def output_Matrix(self):
        return self.box_matrix

    def isRectangleOverlap(self, rec1, rec2):
        x1=rec1['x_min']
        y1=rec1['y_min']
        x2=rec1['x_max']
        y2=rec1['y_max']
        x3 = rec2['x_min']
        y3 = rec2['y_min']
        x4 = rec2['x_max']
        y4 = rec2['y_max']

        return (x4 - x1) * (x2 - x3) > 0 and (y4 - y1) * (y2 - y3) > 0


    def is_below(self,box_below, box_above):
        #下箱体的上平面
        top_surface_box_below = {
            'x_min': box_below.x,
            'x_max': box_below.x + box_below.length,
            'y_min': box_below.y,
            'y_max': box_below.y + box_below.width,
            'z': box_below.z + box_below.height
        }
        # 上箱体的下平面
        bottom_surface_box_above = {
            'x_min': box_above.x,
            'x_max': box_above.x + box_above.length,
            'y_min': box_above.y,
            'y_max': box_above.y + box_above.width,
            'z': box_above.z
        }

        if top_surface_box_below['z'] == bottom_surface_box_above['z'] and self.isRectangleOverlap(top_surface_box_below,bottom_surface_box_above):
            return True

        return False

    def check_stability(self,node):
        if node.child is None:  # 叶子节点
            return True
        return self.is_below(node, node.child) and self.check_stability(node.child)

    #层遍历,树具有一个层次结构，从根节点最高层到最低层，每层从左向右遍历。需要借助队列实现。
    def level_travel(self):
        if self.root == None:
            return

        queue = [self.root]
        while queue:
            cur = node = queue.pop(0)
            print(cur.elem, end=" ")
            if cur.lchild != None:
                queue.append(cur.lchild)
            if cur.rchild != None:
                queue.append(cur.rchild)
    #添加节点
    def add(self, item):
        node = Box_Node(item)

        # 如果节点为空
        if self.root == None:
            self.root = node
            return

        queue = [self.root]
        while queue:
            cur = queue.pop(0)
            if cur.lchild == None:
                cur.lchild = node
                return
            else:
                queue.append(cur.lchild)
            if cur.rchild == None:
                cur.rchild = node
                return
            else:
                queue.append(cur.rchild)

    #先序遍历 先序遍历是先遍历根节点，再遍历左子树，最后遍历右子树。对于遍历子树而言，也是遍历子树的根节点，再遍历子树的左子树，最后遍历子树的右子树。可以用递归实现。
    def preorder_travel(self, node):
        # 如果节点为空
        if node == None:
            return

        print(node.elem, end = " ")
        self.preorder_travel(node.lchild)
        self.preorder_travel(node.rchild)

    #中序遍历 中序遍历则先遍历左子树，再遍历根节点，最后遍历右子树。
    def inorder_travel(self, node):
        # 如果节点为空
        if node == None:
            return
        self.inorder_travel(node.lchild)
        print(node.elem, end=" ")
        self.inorder_travel(node.rchild)

    #后序遍历 后续遍历先遍历左子树，再遍历右子树，最后遍历根节点。
    def postorder_travel(self, node):
        # 如果节点为空
        if node == None:
            return

        self.postorder_travel(node.lchild)
        self.postorder_travel(node.rchild)
        print(node.elem, end=" ")

#找到箱体i上边的所有箱子(不管有无接触)
up_box_array = []

def find_up(box_Matrix,i):
    global up_box_array
    up_box_array.append(i)
    mm_Matrix = box_Matrix[i]
    #找到箱体i上边的所有箱子
    if mm_Matrix[1]!=[]:
        #print(i,"上边的所有箱子序号:",mm_Matrix[1])
        new_ups=mm_Matrix[1]
        for new_up in new_ups:
            up_box_array.append(new_up)
            up_array=find_up(box_Matrix,new_up)

    return up_box_array

#找到箱体i下边的所有箱子(有接触)
def find_down(box_Matrix,i):
    down_box_array = []
    mm_Matrix = box_Matrix[i]
    if mm_Matrix[2]!=[]:
        new_downs = mm_Matrix[2]
        for new_down in new_downs:
            down_box_array.append(new_down)
        return mm_Matrix[2]

    return down_box_array

#计算重合部分矩形
def overlap_rectangle(rec1, rec2):
    x1, y1, x2, y2 = rec1[0], rec1[1], rec1[2], rec1[3]
    x3, y3, x4, y4 = rec2[0], rec2[1], rec2[2], rec2[3]

    # 计算重叠部分的左下角和右上角坐标
    x_left = max(x1, x3)
    y_bottom = max(y1, y3)
    x_right = min(x2, x4)
    y_top = min(y2, y4)

    # 如果不存在重叠部分，则返回一个空列表表示
    if x_right < x_left or y_top < y_bottom:
        return []

    # 构建重叠部分矩形的坐标 [x_left, y_bottom, x_right, y_top]
    overlap_rec = [x_left, y_bottom, x_right, y_top]
    return overlap_rec

def find_all_lines(overlap_rects):
    # 提取矩形的所有角点
    points = []
    for rect in overlap_rects:
        x1, y1, x2, y2 = rect[0], rect[1], rect[2], rect[3]
        points.append((x1, y1))
        points.append((x1, y2))
        points.append((x2, y1))
        points.append((x2, y2))

    # 提取矩形的所有边线段
    lines = []

    # for rect in overlap_rects:
    #     x1, y1, x2, y2 = rect[0], rect[1], rect[2], rect[3]
    #     lines.append(((x1, y1), (x1, y2)))
    #     lines.append(((x2, y1), (x2, y2)))
    #     lines.append(((x1, y1), (x2, y1)))
    #     lines.append(((x1, y2), (x2, y2)))

    # 找到所有角点之间的连接线
    points_list=list(set(points))

    for i in range(len(points_list)):
        for j in range(i + 1, len(points_list)):
            lines.append((points_list[i], points_list[j]))



    return lines

def largest_Polygon(list1):
    number = list1[0]
    x = []
    y = []
    result = ''
    for i in range(1, len(list1)):
        x.append(int(list1[i][0]))
        y.append(int(list1[i][1]))
    index = 0
    min_value = 100
    for i in x:
        if i < min_value:
            min_value_index = index
            min_value = i
        index += 1
    result += str(min_value) + "," + str(y[min_value_index]) + ";"
    status = 1
    max_value1 = 0
    current_x = x[min_value_index]
    current_y = y[min_value_index]
    while 1:
        temp_index =-1
        for i in range(len(list1) - 1):
            if x[i] - current_x > 0 and float((y[i] - current_y)) / (x[i] - current_x + 0.0000001) > max_value1:
                temp_index = i
                max_value1 = float((current_y - y[i])) / (current_x - x[i] + 0.0000001)
        if temp_index != -1:
            current_x = x[temp_index]
            current_y = y[temp_index]
            result += str(current_x) + "," + str(current_y) + ";"
            max_value1 = 0
        else:
            status = 2
            break
    min_value1 = 0
    while 1:
        temp_index =-1
        for i in range(len(list1) - 1):
            if x[i] - current_x > 0 and float((current_y - y[i])) / (current_x - x[i]) < min_value1:
                temp_index = i
                min_value1 = float((current_y - y[i])) / (current_x - x[i])
        if temp_index != -1:
            current_x = x[temp_index]
            current_y = y[temp_index]
            result += str(current_x) + "," + str(current_y) + ";"
            min_value1 = 0
        else:
            status = 3
            break
    max_value2 = 0
    while 1:
        temp_index = -1
        for i in range(len(list1) - 1):
            if x[i] - current_x < 0 and float((current_y - y[i])) / (current_x - x[i]) > max_value2:
                temp_index = i
                max_value2 = float((current_y - y[i])) / (current_x - x[i])
        if temp_index != -1:
            current_x = x[temp_index]
            current_y = y[temp_index]
            result += str(current_x) + "," + str(current_y) + ";"
            max_value2 = 0
        else:
            status = 4
            break
    min_value2 = -10000000
    while 1:
        temp_index =-1
        for i in range(len(list1) - 1):
            if x[i] - current_x < 0 and float((current_y - y[i])) / (current_x - x[i] + 0.000001) > min_value2:
                temp_index = i
                min_value2 = float((current_y - y[i])) / (current_x - x[i] + 0.000001)
        if temp_index !=  -1 and temp_index != min_value_index:
            current_x = x[temp_index]
            current_y = y[temp_index]
            result += str(current_x) + "," + str(current_y) + ";"
            min_value2 = 0
        else:
            break
    return result[:-1]

#判断点 (x, y) 是否落在矩形 (x1, y1, x2, y2) 中
def check_point_in_rect(x, y, x1, y1, x2, y2):
    if x >= x1 and x <= x2 and y >= y1 and y <= y2:
        return True
    return False



#merge的内容暂不需要

# def merge_a_cube(cubeitem,items):
#     all_merge_result = 0
#     merged_cube=[]
#     for j in range(0, len(items)):
#         if can_merge(cubeitem, items[j]):
#             all_merge_result += can_merge(cubeitem, items[j])
#             print("we can merge this cube with ",j,"!")
#             merged_cube = merge(cubeitem, items[j])
#     if all_merge_result==0:
#         return cubeitem
#     else:
#         return merged_cube
#
# def merge_cubes(items):
#     items_new=[]
#     j_list = []
#     all_merge_result=0
#     for i in range(len(items)-1):
#         if_merge = 0
#         if i in j_list:
#             print("dd")
#             if_merge = 1
#         else:
#             for j in range(i+1, len(items)):
#                 if_merge += can_merge(items[i], items[j])
#                 all_merge_result+= can_merge(items[i], items[j])
#                 if can_merge(items[i], items[j]):
#                     print("can merge " ,i," and ",j)
#                     merged_cube = merge(items[i], items[j])
#                     items_new.append(merged_cube)
#                     j_list.append(j)
#                     break
#         print("j_list=",j_list)
#         if if_merge==0:
#             items_new.append(items[i])
#     items_new.append(items[len(items)-1])
#     if all_merge_result==0:
#         return items_new
#     else:
#         items_xin=merge_cubes(items_new)
#         return items_xin
#
# def can_merge(cube1, cube2):
#     x1, y1, z1 = cube1[3], cube1[4], cube1[5]
#     x2, y2, z2 = cube2[3], cube2[4], cube2[5]
#     length1, width1, height1  = cube1[0], cube1[1], cube1[2]
#     length2, width2, height2 = cube2[0], cube2[1], cube2[2]
#
#     if  (z1 == z2 and height1 == height2 and width1 == width2 and y1==y2 and ((x2 == x1 + length1) or (x2 == x1 - length1))):
#         return True
#     return False
#
# def merge(cube1, cube2):
#     x1, y1, z1 = cube1[3], cube1[4], cube1[5]
#     x2, y2, z2 = cube2[3], cube2[4], cube2[5]
#     length1, width1, height1= cube1[0], cube1[1], cube1[2]
#     length2, width2, height2 = cube2[0], cube2[1], cube2[2]
#
#     new_width = width1
#     new_height = height1
#     new_length = length1 +length2
#
#     new_cube = [new_length,new_width, new_height,  min(x1, x2), y1, z1, 0]
#     return new_cube
#


def find_lazy(cubes):
    #print("次数=", len(cubes)-1)
    set_root=[box_w, box_l, 1, 0, 0, -1]
    stu_2=Box_Tree(cubes,set_root)
    stu_2.build_tree()
    M1=stu_2.output_Matrix()

    busy_boxs=[]
    lazy_boxs = []
    for box_idex in range(0,len(cubes)):
        #箱体z=0，判断为稳定
        up_set_box_info=[]
        down_set_box_info=[]

        #print("箱体序号:",box_idex)
        find_up(M1,box_idex)

        up_text=list(set(up_box_array))
        #print("up_boxs=",up_text)

        down_box_array=find_down(M1,box_idex)
        down_text=list(set(down_box_array))

        #print("down_boxs=",down_text)

        for up_idex in up_text:
            up_set_box_info.append(cubes[up_idex])
        for down_idex in down_text:
            down_set_box_info.append(cubes[down_idex])

        #计算i箱体上物品的总体重心
        center_up_boxs = calculate_centroid(up_set_box_info, 1)
        #print(box_idex,"上物品的总体重心位置=",center_up_boxs)

        self_class = Box_Node(cubes[box_idex])
        self_size  = [self_class.x,self_class.y,self_class.x + self_class.length,self_class.y + self_class.width]

        #重合部分矩形，x1, y1, x2, y2
        overlap_rects=[]

        for downs in down_text:
            down_class = Box_Node(cubes[downs])
            down_size = [down_class.x, down_class.y, down_class.x + down_class.length,down_class.y + down_class.width]
            overlap_rect=overlap_rectangle(self_size,down_size)

            #print("box_up=", cubes[box_idex], " box_down = ", cubes[downs])
            #print(box_idex," and ",downs," overlap = ",overlap_rect)
            overlap_rects.append(overlap_rect)

        # 判断上边所有物体重心是否落在本箱体与下边箱体的接触面上
        is_center_on_overlaps=0

        times=0
        for over_rect in overlap_rects:
            is_center_on_overlap_per=check_point_in_rect(center_up_boxs[0],center_up_boxs[1],over_rect[0],over_rect[1],over_rect[2],over_rect[3])
            is_center_on_overlaps+=is_center_on_overlap_per
            if is_center_on_overlap_per:
                busy_boxs.append(down_text[times])#down_text[times]为下边箱子的序号，说明承重了
            times+=1

        # 只要重心满足落在任意1个接触面上即可
        is_center_on_overlap1 = (is_center_on_overlaps != 0)

        #is_center_on_overlap1 为false，说明重心落在外边，这种情况下，下边的所有箱体都承重，发生重心偏移
        if not(is_center_on_overlap1):
            for down_text1 in down_text:
                busy_boxs.append(down_text1)



        #print("up_set_box_info=",up_set_box_info)
        #print("down_set_box_info=",down_set_box_info)

        #列表清空
        up_box_array.clear()
        down_box_array.clear()

    numbers = list(range(0, len(cubes)))

    busy_boxs_result = list(set(busy_boxs))
    lazy_boxs_result = list(set(numbers) - set(busy_boxs_result))


    print("承重箱体:",busy_boxs_result)
    print("非承重箱体:", lazy_boxs_result)

    print("")
    return busy_boxs_result,lazy_boxs_result





def calculate_stability(cubes,heavy_item, light_boxs):
    print("次数=", len(cubes)-1)
    set_root=[box_w, box_l, 1, 0, 0, -1]
    stu_2=Box_Tree(cubes,set_root,heavy_item, light_boxs)
    stu_2.build_tree()
    M1=stu_2.output_Matrix()
    print("箱体矩阵=",M1)


    stable_boxs=[]
    unstable_boxs = []
    for box_idex in range(0,len(cubes)):
        #箱体z=0，判断为稳定
        if cubes[box_idex][5]==0:
            stable_boxs.append(box_idex)
        else:
            up_set_box_info=[]
            down_set_box_info=[]

            print("箱体序号:",box_idex)
            find_up(M1,box_idex)

            up_text=list(set(up_box_array))
            print("up_boxs=",up_text)

            down_box_array=find_down(M1,box_idex)
            down_text=list(set(down_box_array))

            print("down_boxs=",down_text)

            for up_idex in up_text:
                up_set_box_info.append(cubes[up_idex])
            for down_idex in down_text:
                down_set_box_info.append(cubes[down_idex])

            #计算i箱体上物品的总体重心
            center_up_boxs = calculate_centroid(up_set_box_info, 1)
            print(box_idex,"上物品的总体重心位置=",center_up_boxs)

            self_class = Box_Node(cubes[box_idex])
            self_size  = [self_class.x,self_class.y,self_class.x + self_class.length,self_class.y + self_class.width]

            #重合部分矩形，x1, y1, x2, y2
            overlap_rects=[]

            for downs in down_text:
                down_class = Box_Node(cubes[downs])
                down_size = [down_class.x, down_class.y, down_class.x + down_class.length,down_class.y + down_class.width]
                overlap_rect=overlap_rectangle(self_size,down_size)

                print("box_up=", cubes[box_idex], " box_down = ", cubes[downs])
                print(box_idex," and ",downs," overlap = ",overlap_rect)
                overlap_rects.append(overlap_rect)


            # 判断上边所有物体重心是否落在本箱体的身上
            is_center_on_self = check_point_in_rect(center_up_boxs[0], center_up_boxs[1], self_class.x,self_class.y, self_class.x + self_class.length,self_class.y+self_class.width)

            # 判断上边所有物体重心是否落在本箱体与下边箱体的接触面上
            is_center_on_overlaps=0
            for over_rect in overlap_rects:
                is_center_on_overlap_per=check_point_in_rect(center_up_boxs[0],center_up_boxs[1],over_rect[0],over_rect[1],over_rect[2],over_rect[3])
                is_center_on_overlaps+=is_center_on_overlap_per


            #只要重心满足落在任意1个接触面上即可
            is_center_on_overlap=(is_center_on_overlaps!=0)


            # 判断上边所有物体重心是否落在本箱体与下边箱体的接触面集合组成的多边形内
            all_lines=find_all_lines(overlap_rects)
            all_lines_text = list(set(all_lines))



            if box_idex==25:
                print("all_lines_text=",all_lines_text)

            # list1 = [[13], [-3, -3], [1, 3], [2, -4], [6, 1], [-2, -2], [4, 5], [1, -2], [1, 4], [-2, 3], [-4, 1], [-1, 1],
            #          [2, 2], [1, -1]]
            # largest_Polygon_result = largest_Polygon(list1)
            # print("largest_Polygon_result=", largest_Polygon_result)

            if is_center_on_self and is_center_on_overlap:
                stable_boxs.append(box_idex)
            else:
                unstable_boxs.append(box_idex)





            print("up_set_box_info=",up_set_box_info)
            print("down_set_box_info=",down_set_box_info)

            #列表清空
            up_box_array.clear()
            down_box_array.clear()

    print("稳定箱体:",stable_boxs)
    print("不稳定箱体:", unstable_boxs)
    print("")
    return stable_boxs,unstable_boxs





def add_texture_on_cube(box, texture):
    DX, DY = 0.5 / 2, 0.66 / 2
    one = [[3 * DX, 1 * DY], [3 * DX, 2 * DY], [4 * DX, 2 * DY], [4 * DX, 2 * DY], [4 * DX, 1 * DY], [3 * DX, 1 * DY]]
    two = [[2 * DX, 1 * DY], [2 * DX, 2 * DY], [3 * DX, 2 * DY], [3 * DX, 2 * DY], [3 * DX, 1 * DY], [2 * DX, 1 * DY]]
    three = [[1 * DX, 1 * DY], [1 * DX, 2 * DY], [2 * DX, 2 * DY], [2 * DX, 2 * DY], [2 * DX, 1 * DY], [1 * DX, 1 * DY]]
    four = [[0 * DX, 1 * DY], [0 * DX, 2 * DY], [1 * DX, 2 * DY], [1 * DX, 2 * DY], [1 * DX, 1 * DY], [0 * DX, 1 * DY]]
    five = [[1 * DX, 0 * DY], [1 * DX, 1 * DY], [2 * DX, 1 * DY], [2 * DX, 1 * DY], [2 * DX, 0 * DY], [1 * DX, 0 * DY]]
    six = [[1 * DX, 2 * DY], [1 * DX, 3 * DY], [2 * DX, 3 * DY], [2 * DX, 3 * DY], [2 * DX, 2 * DY], [1 * DX, 2 * DY]]
    v_uv = np.concatenate((one, two, three, four, five, six), axis=0)
    box.triangle_uvs = o3d.open3d.utility.Vector2dVector(v_uv)
    faces = np.array(box.triangles)
    box.triangle_material_ids = o3d.utility.IntVector([0] * len(faces))
    box.textures = [o3d.geometry.Image(texture)]
    return box


def create_container(dimensions):
    length = dimensions[0]
    width = dimensions[1]
    height = dimensions[2]
    x = dimensions[3]
    y = dimensions[4]
    z = dimensions[5]
    vertices = [
        (x, y, z),
        (x+length, y, z),
        (x+length, y+width, z),
        (x, y+width, z),
        (x, y, z+height),
        (x+length, y, z+height),
        (x+length, y+width, z+height),
        (x, y+width, z+height)
    ]
    faces = [[3, 0, 1], [1, 2, 3],
             [1, 5, 6], [6, 2, 1],
             [5, 4, 7], [7, 6, 5],
             [4, 0, 3], [3, 7, 4],
             [6, 7, 3], [3, 2, 6],
             [1, 0, 4], [4, 5, 1]]

    verts_vector = o3d.open3d.utility.Vector3dVector(vertices)
    faces_vector = o3d.open3d.utility.Vector3iVector(faces)


    container = o3d.geometry.TriangleMesh(verts_vector, faces_vector)

    container.compute_vertex_normals()




    lines = np.array([[0, 1], [1, 2], [2, 3], [3, 0],
                      [4, 5], [5, 6], [6, 7], [7, 4],
                      [0, 4], [1, 5], [2, 6], [3, 7]])
    line_color = [0, 0, 0]
    black1 = np.tile(line_color, (lines.shape[0], 1))
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(vertices)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(black1)

    container.paint_uniform_color((255,255,255) ) # 设置立方体的颜色为传入的color参数
    #container.compute_vertex_normals()
    return container,line_set

def create_cube1(dimensions,color):
    length = dimensions[0]
    width = dimensions[1]
    height = dimensions[2]
    x = dimensions[3]
    y = dimensions[4]
    z = dimensions[5]

    vertices = [
        (x, y, z),
        (x+length, y, z),
        (x+length, y+width, z),
        (x, y+width, z),
        (x, y, z+height),
        (x+length, y, z+height),
        (x+length, y+width, z+height),
        (x, y+width, z+height)
    ]

    faces = [[3, 0, 1], [1, 2, 3],
             [1, 5, 6], [6, 2, 1],
             [5, 4, 7], [7, 6, 5],
             [4, 0, 3], [3, 7, 4],
             [6, 7, 3], [3, 2, 6],
             [1, 0, 4], [4, 5, 1]]

    verts_vector = o3d.open3d.utility.Vector3dVector(vertices)
    faces_vector = o3d.open3d.utility.Vector3iVector(faces)
    container = o3d.geometry.TriangleMesh(verts_vector, faces_vector)
    container.paint_uniform_color(color)  # 设置立方体的颜色为传入的color参数
    container.compute_vertex_normals()

    lines = np.array([[0, 1], [1, 2], [2, 3], [3, 0],
                      [4, 5], [5, 6], [6, 7], [7, 4],
                      [0, 4], [1, 5], [2, 6], [3, 7]])

    line_color = [0, 0, 0]
    black1 = np.tile(line_color, (lines.shape[0], 1))

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(vertices)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(black1)

    return container,line_set


def create_cube2(dimensions, color):
    length = dimensions[0]
    width = dimensions[1]
    height = dimensions[2]
    x = dimensions[3]
    y = dimensions[4]
    z = dimensions[5]

    vertices = [
        (x, y, z),
        (x + length, y, z),
        (x + length, y + width, z),
        (x, y + width, z),
        (x, y, z + height),
        (x + length, y, z + height),
        (x + length, y + width, z + height),
        (x, y + width, z + height)
    ]

    lines = np.array([[0, 1], [1, 2], [2, 3], [3, 0],
                      [4, 5], [5, 6], [6, 7], [7, 4],
                      [0, 4], [1, 5], [2, 6], [3, 7]])

    line_color = [0, 0, 0]
    black1 = np.tile(line_color, (lines.shape[0], 1))

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(vertices)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(black1)


    cube1 = o3d.geometry.TriangleMesh.create_box(dimensions[0], dimensions[1], dimensions[2])


    cube1.translate([dimensions[3], dimensions[4], dimensions[5]])
    cube1.paint_uniform_color(color)  # 设置立方体的颜色为传入的color参数

    return cube1, line_set

def create_build_pic(pic_in, width,height,color):
    my_array = [height, width]
    my_sort = sorted(my_array)
    size_S =my_sort[0]*100
    imagetest = np.ones((height*100, width*100, 3), dtype=np.uint8)
    imagetest[:, :] = [color[2] * 255.0, color[1] * 255.0, color[0] * 255.0]
    pilimg1 = Image.fromarray(imagetest)
    pic_out_tmp = cv2.cvtColor(np.array(pilimg1), cv2.COLOR_RGB2BGR)
    resized_image = cv2.resize(pic_in, (size_S, size_S))
    x_offset = (width* 100 / 2 - size_S / 2)
    y_offset = (height * 100 / 2 - size_S / 2)
    pic_out_tmp[int(y_offset):int(y_offset) + int(size_S), int(x_offset):int(x_offset) + int(size_S)] = resized_image

    pic_out = cv2.resize(pic_out_tmp, (1000, 1000))

    return pic_out


def create_cube(dimensions, color,cube_num):
    length = dimensions[0]
    width = dimensions[1]
    height = dimensions[2]
    x = dimensions[3]
    y = dimensions[4]
    z = dimensions[5]

    vertices = [
        (x, y, z),
        (x+length, y, z),
        (x+length, y+width, z),
        (x, y+width, z),
        (x, y, z+height),
        (x+length, y, z+height),
        (x+length, y+width, z+height),
        (x, y+width, z+height)
    ]

    lines = np.array([[0, 1], [1, 2], [2, 3], [3, 0],
                      [4, 5], [5, 6], [6, 7], [7, 4],
                      [0, 4], [1, 5], [2, 6], [3, 7]])

    line_color = [0, 0, 0]
    black1 = np.tile(line_color, (lines.shape[0], 1))

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(vertices)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(black1)

    vertsQ = [[0, 0, 0], [0, width, 0], [length, width, 0], [length, 0, 0],
            [0, 0, height], [0, width, height], [length, width, height], [length, 0, height]]
    facesQ = [[3, 0, 1], [1, 2, 3],
             [1, 5, 6], [6, 2, 1],
             [5, 4, 7], [7, 6, 5],
             [4, 0, 3], [3, 7, 4],
             [6, 7, 3], [3, 2, 6],
             [1, 0, 4], [4, 5, 1]]


    cube = o3d.geometry.TriangleMesh(o3d.open3d.utility.Vector3dVector(vertsQ),
                                  o3d.open3d.utility.Vector3iVector(facesQ))


    size_zi=1000
    text_num=str(cube_num)
    width1 = size_zi
    height1 = size_zi*len(text_num)//2
    imager = np.ones((width1, height1, 3), dtype=np.uint8)
    imager[:, :] = [color[2]*255.0, color[1]*255.0,color[0]*255.0]



    pilimg = Image.fromarray(imager)
    draw = ImageDraw.Draw(pilimg)  # 图片上打印
    font = ImageFont.truetype("simhei.ttf", size_zi, encoding="utf-8")  # 参数1：字体文件路径，参数2：字体大小
    draw.text((0, 0), text_num, (0,0,0), font=font)  # 参数1：打印坐标，参数2：文本，参数3：字体颜色，参数4：字体
    cv2charimg = cv2.cvtColor(np.array(pilimg), cv2.COLOR_RGB2BGR)


    img36=create_build_pic(cv2charimg,length,width,color)
    img24 = create_build_pic(cv2charimg, width, height, color)
    img15 = create_build_pic(cv2charimg, length, height, color)

    imagetest_test = np.ones((1000, 1000, 3), dtype=np.uint8)
    imagetest_test[:, :] = [0,0,0]


    imagefull = np.ones((3000, 4000, 3), dtype=np.uint8)
    imagefull[:, :] = [255,255,255]


    # 将图像顺时针旋转90度
    rotated_img36 = cv2.rotate(img36, cv2.ROTATE_90_CLOCKWISE)

    CLOCKWISE_img24 = cv2.rotate(img24, cv2.ROTATE_90_CLOCKWISE)
    #将图像顺时针旋转270度或逆时针旋转90度
    COUNTERCLOCKWISE_img24 = cv2.rotate(img24, cv2.ROTATE_90_COUNTERCLOCKWISE)

    # 将图像顺时针旋转180度
    rotated_img15 = cv2.rotate(img15, cv2.ROTATE_180)



    imagefull[1000:2000, 0000:1000] = img15#1
    imagefull[0000:1000, 1000:2000] = COUNTERCLOCKWISE_img24#2
    imagefull[1000:2000, 1000:2000] = img36#3
    imagefull[2000:3000, 1000:2000] = CLOCKWISE_img24#4
    imagefull[1000:2000, 2000:3000] = rotated_img15#5
    imagefull[1000:2000, 3000:4000] = rotated_img36#6

    DX, DY = 0.5 / 2, 0.66 / 2  # UV坐标

    one = [[3 * DX, 1 * DY], [3 * DX, 2 * DY], [4 * DX, 2 * DY], [4 * DX, 2 * DY], [4 * DX, 1 * DY], [3 * DX, 1 * DY]]
    two = [[2 * DX, 1 * DY], [2 * DX, 2 * DY], [3 * DX, 2 * DY], [3 * DX, 2 * DY], [3 * DX, 1 * DY], [2 * DX, 1 * DY]]
    three = [[1 * DX, 1 * DY], [1 * DX, 2 * DY], [2 * DX, 2 * DY], [2 * DX, 2 * DY], [2 * DX, 1 * DY], [1 * DX, 1 * DY]]
    four = [[0 * DX, 1 * DY], [0 * DX, 2 * DY], [1 * DX, 2 * DY], [1 * DX, 2 * DY], [1 * DX, 1 * DY], [0 * DX, 1 * DY]]
    five = [[1 * DX, 0 * DY], [1 * DX, 1 * DY], [2 * DX, 1 * DY], [2 * DX, 1 * DY], [2 * DX, 0 * DY], [1 * DX, 0 * DY]]
    six = [[1 * DX, 2 * DY], [1 * DX, 3 * DY], [2 * DX, 3 * DY], [2 * DX, 3 * DY], [2 * DX, 2 * DY], [1 * DX, 2 * DY]]


    v_uv = np.concatenate((one, two, three, four, five, six), axis=0)
    cube.triangle_uvs = o3d.open3d.utility.Vector2dVector(v_uv)
    cube.triangle_material_ids = o3d.utility.IntVector([0] * len(facesQ))
    cube.textures = [o3d.geometry.Image(imagefull)]



    cube.translate([dimensions[3], dimensions[4], dimensions[5]])
    #cube.paint_uniform_color(color)  # 设置立方体的颜色为传入的color参数

    return cube,line_set

#显示重心
def create_WeightPoint(x,y,z, ball_color,size):

    # 创建几何对象
    ball_vis = o3d.geometry.TriangleMesh.create_sphere(radius=size)
    ball_vis.paint_uniform_color(ball_color)  # 设置立方体的颜色为传入的color参数
    ball_vis.translate([x,y,z])

    return ball_vis


# 计算重心
def calculate_center(item):
    length = item[0]
    width = item[1]
    height = item[2]
    x = item[3]
    y = item[4]
    z = item[5]
    # 计算立方体顶点坐标
    vertices = [[x, y, z],
                [x + length, y, z],
                [x + length, y + width, z],
                [x, y + width, z],
                [x, y, z + height],
                [x + length, y, z + height],
                [x + length, y + width, z + height],
                [x, y + width, z + height]]

    # 计算中心点坐标
    center_x = sum([vertex[0] for vertex in vertices]) / 8
    center_y = sum([vertex[1] for vertex in vertices]) / 8
    center_z = sum([vertex[2] for vertex in vertices]) / 8

    return [center_x, center_y, center_z]




#
# items0 = [[6, 7, 4, 6, 4, 0, 0], [6, 7, 4, 0, 4, 0, 0], [6, 7, 4, 6, 4, 4, 0], [6, 7, 4, 0, 4, 4, 0],
#              [6, 7, 4, 0, 4, 8, 0], [6, 7, 4, 6, 4, 8, 0], [6, 7, 4, 6, 4, 12, 0], [6, 7, 4, 0, 4, 12, 0],
#              [11, 2, 2, 1, 2, 0, 0], [11, 2, 2, 1, 0, 0, 0], [11, 2, 2, 1, 0, 2, 0], [11, 2, 2, 1, 2, 2, 0],
#              [11, 2, 2, 1, 0, 4, 0], [11, 2, 2, 1, 2, 4, 0], [11, 2, 2, 1, 0, 6, 0], [11, 2, 2, 1, 2, 6, 0],
#              [8, 3, 1, 4, 0, 8, 0], [8, 3, 1, 4, 0, 9, 0], [8, 3, 1, 4, 1, 10, 0], [8, 3, 1, 4, 1, 11, 0],
#              [8, 3, 1, 4, 1, 12, 0], [3, 8, 1, 9, 3, 16, 0], [3, 8, 1, 6, 3, 16, 0], [3, 8, 1, 3, 3, 16, 0],
#              [3, 4, 3, 0, 0, 8, 0], [3, 4, 3, 0, 0, 11, 0], [4, 3, 3, 8, 0, 13, 0], [4, 3, 3, 4, 0, 13, 0],
#              [3, 4, 3, 0, 0, 14, 0], [3, 4, 3, 0, 7, 16, 0], [3, 4, 3, 9, 7, 17, 0], [3, 4, 3, 6, 7, 17, 0]]

# items0 = [[6, 7, 4, 6, 4, 0, 0], [2, 11, 2, 4, 0, 0, 0], [3, 8, 1, 1, 3, 0, 0], [3, 4, 3, 9, 0, 0, 0],
#              [6, 7, 4, 6, 4, 4, 0], [2, 11, 2, 2, 0, 1, 0], [3, 8, 1, 9, 3, 8, 0], [3, 4, 3, 6, 0, 0, 0],
#              [6, 7, 4, 0, 4, 3, 0], [2, 11, 2, 4, 0, 7, 0], [3, 8, 1, 1, 3, 7, 0], [3, 4, 3, 6, 0, 3, 0],
#              [7, 6, 4, 5, 5, 9, 0], [2, 11, 2, 2, 0, 8, 0], [3, 8, 1, 2, 3, 10, 0], [3, 4, 3, 6, 0, 6, 0]]
items0=[]

items1=[
[10,5,3,0,0,0,0],
    [10,6,1,0,5,0,0],
    [2,11,2,10,0,0,0],
    [8,3,1,0,7,1,0],
    [9,2,2,0,5,1,0],
    [4,3,3,0,7,2,0],
    [2,11,1,10,0,2,0],
    [6,7,4,0,0,3,0],
    [6,11,1,6,0,3,0],
    [6,7,3,6,0,4,0],
    [5,4,3,6,7,4,0],
    [2,8,2,0,0,7,0],
    [5,11,2,2,0,7,0],
    [4,8,6,7,0,7,0],
    [6,7,4,0,0,9,0]

]


# items1=[
# [8,4,6,0,0,0,0],
# [3,4,3,8,0,0,0],
# [6,7,4,0,4,0,0],
# [6,7,4,6,4,0,0],
#
# [11,5,2,0,4,4,0],
#
# [11,2,2,0,9,4,0],
# [6,11,1,0,0,6,0],
#
# [6,10,1,6,0,6,0],
# [10,5,3,0,0,7,0],
#
# [7,6,3,0,5,7,0],
# [4,5,3,7,5,7,0],
#
# [2,9,2,0,0,10,0],
# [2,8,2,2,0,10,0],
# [3,8,1,4,0,10,0],
# [11,2,1,0,9,10,0]
# ]

# items1=[
# [6,	7,	4 ,0,0  ,1-1,0],
# [4,	3,	3,0,0   ,5-1,0],
# [5,	4,	3,0     ,3,5-1,0],
# [8,	4,	6,0     ,7,1-1,0],
# [9,	2,	2,0     ,7,7-1,0],
# [8,	2,	2,0     ,7,9-1,0],
# [6,	7,	3,6     ,0,1-1,0],
# [6,	7,	4,6     ,0,4-1,0],
# [11,2,	2,0     ,0,8-1,0],
# [11,2,	1,0     ,0,10-1,0],
# [10,5,	3,0     ,2,8-1,0],
# [11,5,	2,0     ,0,11-1,0],
# [11,6,	1,0     ,0,13-1,0],
# [10,6,	1,0     ,0,14-1,0],
# [8,	3,	1,0     ,5,11-1,0]
# ]


# items1 =[[5  	,11  	,2  ,0  	,0  	,0  , 0],
# [6  	,11  	,1  ,5  ,	0  	,0  , 0	],
# [2  ,11  ,2  ,5  	,0  	,1  , 0],
# [2  	,9  	,2  ,7  ,0  ,1  ,	 0],
# [2  	,8  	,2  ,9  ,	0  	,1  	, 0],
# [2  	,11  	,1  ,0  ,	0  	,2  	, 0],
# [3  	,8  	,1  ,2  ,	0  	,2  	, 0],
# [7  	,6  	,3  ,0  ,	0  	,3  	, 0],
# [10  	,5  	,3  ,0  ,	6  ,	3  	, 0],
# [4  ,	5  ,	3  ,7  ,	0  	,3  	, 0],
# [6  ,	7  ,	4  ,0  ,	0  	,6  	, 0],
# [8  ,	4  ,	6  ,0  ,	7  ,	6  ,	 0],
# [6  ,	7  ,	4  , 6  ,	0  ,	6  ,	0],
# [3  ,	4  ,	3  ,8  ,	7  ,	6  ,	 0],
# [10  ,	6  ,	1  , 0  ,	0  ,	10  ,	0]]

# items1 =  [[2, 9, 2, 0, 2, 0, 0], [6, 10, 1, 6, 1, 0, 0], [2, 8, 2, 2, 3, 0, 0],
#            [6, 11, 1, 6, 0, 1, 0], [6, 7, 3, 6, 4, 2, 0], [4, 5, 3, 0, 6, 2, 0],
#            [2, 11, 2, 4, 0, 0, 0], [8, 3, 1, 4, 0, 2, 0], [3, 4, 3, 0, 2, 2, 0],
#            [6, 7, 4, 0, 4, 5, 0], [6, 7, 4, 6, 4, 5, 0], [8, 4, 6, 4, 0, 3, 0],
#            [5, 10, 3, 0, 1, 9, 0], [2, 11, 1, 10, 0, 9, 0], [5, 11, 2, 5, 0, 9, 0]]

# items1 = [[3, 4, 3, 9, 0, 0, 0], [3, 4, 3, 9, 7, 0, 0], [3, 4, 3, 6, 7, 0, 0], [3, 4, 3, 6, 3, 0, 0],
#          [3, 4, 3, 3, 7, 0, 0], [3, 4, 3, 3, 3, 0, 0], [3, 4, 3, 0, 7, 0, 0], [3, 4, 3, 0, 3, 0, 0],
#          [8, 3, 1, 1, 0, 0, 0], [3, 8, 1, 9, 3, 3, 0], [3, 8, 1, 6, 3, 3, 0], [3, 8, 1, 3, 3, 3, 0],
#          [3, 8, 1, 0, 3, 3, 0], [8, 3, 1, 1, 0, 1, 0], [3, 8, 1, 0, 3, 4, 0], [3, 8, 1, 3, 3, 4, 0],
#          [2, 11, 2, 6, 0, 4, 0], [2, 11, 2, 8, 0, 4, 0], [2, 11, 2, 0, 0, 5, 0], [2, 11, 2, 2, 0, 5, 0],
#          [2, 11, 2, 4, 0, 5, 0], [2, 11, 2, 10, 0, 4, 0], [2, 11, 2, 6, 0, 6, 0], [2, 11, 2, 8, 0, 6, 0],
#          [6, 7, 4, 0, 4, 7, 0], [6, 7, 4, 6, 4, 8, 0], [6, 7, 4, 0, 4, 11, 0], [6, 7, 4, 6, 4, 12, 0],
#          [6, 7, 4, 0, 4, 15, 0], [6, 7, 4, 6, 4, 16, 0]]

# items1 = [[5 , 11 , 2 , 0 , 0 , 0 , 0],
#        [6 , 11 , 1 , 5 , 0 , 0 , 0],
#        [2 , 11 , 2 , 5 , 0 , 1 , 0],
#        [2 , 9 , 2 , 7 , 0 , 1 , 0],
#        [2 , 8 , 2 , 9 , 0 , 1 , 0],
#        [2 , 11 , 1 , 0 , 0 , 2 , 0],
#        [3 , 8 , 1 , 2 , 0 , 2 , 0],
#        [7 , 6 , 3 , 0 , 0 , 3 , 0],
#        [10 , 5 , 3 , 0 , 6 , 3 , 0],
#        [4 , 5 , 3 , 7 , 0 , 3 , 0],
#        [6 , 7 , 4 , 0 , 0 , 6 , 0],
#        [8 , 4 , 6 , 0 , 7 , 6 , 0],
#        [6 , 7 , 4 , 6 , 0 , 6 , 0],
#        [3 , 4 , 3 , 8 , 7 , 6 , 0],
#        [10 , 6 , 1 , 0 , 0 , 10 , 0]]

# items0 = [[3, 8, 1, 9, 0, 0, 0], [2, 11, 2, 7, 0, 0, 0], [6, 7, 4, 1, 4, 0, 0], [3, 4, 3, 4, 0, 0, 0],
#          [3, 4, 3, 9, 0, 1, 0], [2, 11, 2, 7, 0, 2, 0], [3, 8, 1, 0, 3, 4, 0], [2, 11, 2, 3, 0, 4, 0],
#          [2, 11, 2, 5, 0, 4, 0], [7, 6, 4, 0, 5, 6, 0], [3, 8, 1, 7, 3, 4, 0], [2, 11, 2, 7, 0, 5, 0],
#          [3, 4, 3, 4, 1, 6, 0], [3, 4, 3, 7, 7, 7, 0], [6, 7, 4, 0, 4, 10, 0], [7, 6, 4, 0, 5, 14, 0],
#          [3, 8, 1, 0, 3, 18, 0], [3, 8, 1, 3, 3, 18, 0]]


# items0 = [[3, 4, 3, 9, 0, 0, 0], [3, 8, 1, 6, 3, 0, 0], [6, 7, 4, 0, 4, 0, 0], [6, 7, 4, 0, 4, 4, 0],
#          [2, 11, 2, 7, 0, 1, 0], [2, 11, 2, 0, 0, 8, 0], [2, 11, 2, 2, 0, 8, 0], [3, 4, 3, 9, 7, 0, 0],
#          [3, 8, 1, 9, 3, 3, 0], [6, 7, 4, 0, 4, 10, 0], [3, 4, 3, 6, 7, 3, 0], [3, 8, 1, 9, 3, 4, 0],
#          [2, 11, 2, 9, 0, 5, 0], [3, 4, 3, 6, 3, 3, 0], [3, 8, 1, 6, 3, 6, 0], [3, 8, 1, 6, 3, 7, 0],
#          [3, 4, 3, 9, 7, 7, 0], [2, 11, 2, 6, 0, 8, 0], [6, 7, 4, 6, 4, 10, 0], [6, 7, 4, 0, 4, 14, 0]]
#

# items0 = [[3, 5, 4, 3, 2, 0, 0],
#           [5, 4, 1, 1, 3, 4, 0],
#           [5, 1, 2, 0, 1, 0, 0],
#           [1, 1, 4, 5, 2, 4, 0],
#           [4, 4, 1, 2, 3, 5, 0],
#           [3, 4, 1, 3, 3, 6, 0],
#           [2, 2, 2, 1, 1, 2, 0],
#           [1, 5, 1, 2, 0, 6, 0],
#           [3, 2, 3, 3, 0, 2, 0],
#           [1, 3, 4, 1, 0, 4, 0],
#           [1, 4, 2, 1, 3, 5, 0]]

k_time = 0
last_update_time = 0  # 上一次更新显示的时间

def rgbtohsi(rgb_lwpImg):
    rows = int(rgb_lwpImg.shape[0])
    cols = int(rgb_lwpImg.shape[1])
    b, g, r = cv2.split(rgb_lwpImg)
    # 归一化到[0,1]
    b = b / 255.0
    g = g / 255.0
    r = r / 255.0
    hsi_lwpImg = rgb_lwpImg.copy()
    H, S, I = cv2.split(hsi_lwpImg)
    for i in range(rows):
        for j in range(cols):
            num = 0.5 * ((r[i, j]-g[i, j])+(r[i, j]-b[i, j]))
            den = np.sqrt((r[i, j]-g[i, j])**2+(r[i, j]-b[i, j])*(g[i, j]-b[i, j]))
            theta = float(np.arccos(num/den))

            if den == 0:
                    H = 0
            elif b[i, j] <= g[i, j]:
                H = theta
            else:
                H = 2*3.14169265 - theta

            min_RGB = min(min(b[i, j], g[i, j]), r[i, j])
            sum = b[i, j]+g[i, j]+r[i, j]
            if sum == 0:
                S = 0
            else:
                S = 1 - 3*min_RGB/sum

            H = H/(2*3.14159265)
            I = sum/3.0
            # 输出HSI图像，扩充到255以方便显示，一般H分量在[0,2pi]之间，S和I在[0,1]之间
            hsi_lwpImg[i, j, 0] = H*255
            hsi_lwpImg[i, j, 1] = S*255
            hsi_lwpImg[i, j, 2] = I*255
    return hsi_lwpImg

def get_n_hls_colors(nums):
    hls_colors = []
    step=360.0/nums
    for num in range(nums):
        h=step*num
        s=90+random.random()*10
        l=50+random.random()*10
        _hls = [h/360.0,l/100.0,s/100.0]

        hls_colors.append(_hls)

    return hls_colors

def ncolors(num):
    rgb_colors = []
    hls_colors = get_n_hls_colors(num)
    for hlsc in hls_colors:
        _r, _g, _b = colorsys.hls_to_rgb(hlsc[0], hlsc[1], hlsc[2])
        r,g,b=[(x) for x in (_r,_g,_b)]
        rgb_colors.append([r,g,b])
    return rgb_colors


def modify_image(inmat,S_c,V_c):
    # 将图片修改为HSV
    pichsv = cv2.cvtColor(inmat, cv2.COLOR_BGR2HSV)
    # 提取饱和度和明度
    H,S,V = cv2.split(pichsv)
    # S为饱和度，V为明度
    new_pic = cv2.merge([np.uint8(H), np.uint8(S*S_c), np.uint8(V*V_c)])
    # 将合并后的图片重置为RGB
    pictar = cv2.cvtColor(new_pic, cv2.COLOR_HSV2BGR)
    return pictar

def get_colors(name, lut):
    return plt.get_cmap(name, lut)([i for i in range(lut)])
# """
# params:
# 	- name：颜色图谱，可以是字符串，也可以是colormap实例
# 	- lut：要得到的颜色个数，一个整数
# """

#计算整体中心
def calculate_centroid(cubes, density):
    total_volume = 0
    weighted_centroid = [0, 0, 0]
    for cube in cubes:
        length, width, height, x, y, z, _ = cube
        volume = length * width * height
        centroid = [x + (length / 2), y + (width / 2), z + (height / 2)]
        total_volume += volume
        weighted_centroid[0] += volume * centroid[0]
        weighted_centroid[1] += volume * centroid[1]
        weighted_centroid[2] += volume * centroid[2]
  #      print("centroid=",centroid)
  #  print("total_volume=",total_volume)
    overall_centroid = [coord / (total_volume) for coord in weighted_centroid]
    return overall_centroid

def my_calculate_centroid(cubes, density):
    centers = [calculate_center(item) for item in items0]
#    print(centers)
    m_total=MiXi_total=MiYi_total=MiZi_total=0.0


    for i in range(len(centers)):
        Mi=cubes[i][0]*cubes[i][1]*cubes[i][2]*density
        MiXi=Mi*centers[i][0]
        MiYi=Mi*centers[i][1]
        MiZi=Mi*centers[i][2]
        m_total+=Mi
        MiXi_total+=MiXi
        MiYi_total+=MiYi
        MiZi_total+=MiZi
    X=MiXi_total/m_total
    Y=MiYi_total/m_total
    Z=MiZi_total/m_total
   # print("m_total=",m_total)
    return X,Y,Z
def extract_rows(items, n):
    extracted_rows = items[0:n]
    return extracted_rows


color_list=[]
use_rate = 0.0
weight_center_height=0.0
balance_boxs=[]
unbalanced_boxs=[]

cubes_Total=[]
lines_Total=[]

def update_callback_no_interval(vis,isdel=0):
    global k_time, last_update_time,color_list,use_rate,param_before,weight_center_height,balance_boxs,unbalanced_boxs
    print("k_time=",k_time)
    if (k_time ==0) and isdel==0:
        # 创建和添加坐标轴
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=3.0, origin=[0, 0, 0])
        container_size = [box_w, box_l, box_h, 0, 0, 0, 0]
        container, container_line_set = create_container(container_size)
        vis.add_geometry(coord_frame)
        vis.add_geometry(container)
        vis.add_geometry(container_line_set)
        opt = vis.get_render_option()
        opt.background_color = np.asarray([1, 1, 1])
        ctraa = vis.get_view_control()
        ctraa.set_lookat(np.array([box_w/2, box_l/2, box_h/2]))
        ctraa.set_zoom(1.2)
        ctraa.set_up((0, 0, 1))  # set the positive direction of the x-axis as the up direction
        ctraa.set_front((1, 0, 0))  # set the positive direction of the x-axis toward you

    ctr_before = vis.get_view_control()
    param_before = ctr_before.convert_to_pinhole_camera_parameters()



    if (k_time < len(items0)):
        #print(" k_time=  ",k_time,"  len(items0)=  ",len(items0)," items1[k_time]=  ",items0[k_time])
        # 更新Cube和Line Set数据
        if isdel == 0:
            # colors = (color_list[k_time][0], color_list[k_time][1], color_list[k_time][2])
            # cube1, line_set1 = create_cube(items0[k_time],colors,k_time)
            # cubes_Total.append(cube1)
            # lines_Total.append(line_set1)
            vis.add_geometry(cubes_Total[k_time])
            vis.add_geometry(lines_Total[k_time])
            # vis.update_geometry(cube1)
            # vis.update_geometry(line_set1)

        else:
            vis.remove_geometry(cubes_Total[k_time+1])
            vis.remove_geometry(lines_Total[k_time+1])
            # cubes_Total.pop()
            # lines_Total.pop()

        if (k_time == -1):
            use_rate=0
            weight_center_height=0
        else:
            ctr_after = vis.get_view_control()
            ctr_after.convert_from_pinhole_camera_parameters(param_before)
            V_total = 0.0
            H_list = []
            X_list = []
            Y_list = []

            extracted = extract_rows(items0, k_time+1)
            weight_center = calculate_centroid(extracted, 1)
            weight_center_height = weight_center[2]

            for cal_VV in extracted:
                #计算每个箱体重心
                V_per = cal_VV[0] * cal_VV[1] * cal_VV[2]
                V_total += V_per
                X_list.append(cal_VV[0] + cal_VV[3])
                Y_list.append(cal_VV[1] + cal_VV[4])
                H_list.append(cal_VV[2] + cal_VV[5])
            #计算利用率
            use_rate = V_total / (max(H_list) * box_w * box_l)

        k_time += 1

    else:
        return False



    return True



def update_callback(vis,time_interval):
    global k_time, last_update_time,color_list,use_rate,param_before,weight_center_height,balance_boxs,unbalanced_boxs

    if (k_time ==0):
        # 创建和添加坐标轴
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=3.0, origin=[0, 0, 0])
        container_size = [box_w, box_l, box_h, 0, 0, 0, 0]
        container, container_line_set = create_container(container_size)


        #add_texture_on_cube(container, img)

        vis.add_geometry(coord_frame)

        vis.add_geometry(container)
        vis.add_geometry(container_line_set)

        opt = vis.get_render_option()
        opt.background_color = np.asarray([1, 1, 1])
        ctraa = vis.get_view_control()
        ctraa.set_lookat(np.array([box_w/2, box_l/2, box_h/2]))
        ctraa.set_zoom(1.2)
        ctraa.set_up((0, 0, 1))  # set the positive direction of the x-axis as the up direction
        ctraa.set_front((1, 0, 0))  # set the positive direction of the x-axis toward you



    ctr_before = vis.get_view_control()
    param_before = ctr_before.convert_to_pinhole_camera_parameters()
    current_time = time.time()  # 当前系统时间

    if (k_time < len(items0)):
        if (current_time - last_update_time >= time_interval):

            #print(" k_time=  ",k_time,"  len(items0)=  ",len(items0)," items1[k_time]=  ",items0[k_time])
            # 更新Cube和Line Set数据
            # colors = (color_list[k_time][0], color_list[k_time][1], color_list[k_time][2])
            # cube1, line_set1 = create_cube(items0[k_time],colors,k_time)
            # cubes_Total.append(cube1)
            # lines_Total.append(line_set1)

            vis.add_geometry(cubes_Total[k_time])
            vis.add_geometry(lines_Total[k_time])

            # vis.add_geometry(cube1)
            # vis.add_geometry(line_set1)
            # vis.update_geometry(cubes_Total[k_time])
            # vis.update_geometry(lines_Total[k_time])




            ctr_after = vis.get_view_control()
            ctr_after.convert_from_pinhole_camera_parameters(param_before)

            V_total = 0.0
            H_list = []
            X_list = []
            Y_list = []

            extracted = extract_rows(items0, k_time+1)

            #print("extracted_cubes=",extracted)

            for cal_VV in extracted:
                #计算每个箱体重心
                weight_center = calculate_centroid(extracted, 1)
                weight_center_height=weight_center[2]
                V_per = cal_VV[0] * cal_VV[1] * cal_VV[2]
                V_total += V_per
                X_list.append(cal_VV[0] + cal_VV[3])
                Y_list.append(cal_VV[1] + cal_VV[4])
                H_list.append(cal_VV[2] + cal_VV[5])

            # # 计算稳定度
            # busy_boxs, lazy_boxs = find_lazy(extracted)
            # balance_boxs, unbalanced_boxs = calculate_stability(extracted,busy_boxs, lazy_boxs)

            #计算利用率
            use_rate = V_total / (max(H_list) * box_w * box_l)

            # print("extracted=",extracted)
            # print("V_total=", V_total)
            # print("max(H_list)=", max(H_list))
            # print("box_w=",box_w)
            # print("box_l=",box_l)
            # print("use_rate=",use_rate)
            k_time+=1
            last_update_time = current_time  # 更新上一次更新的时间
        else:
            pass
    else:
        return False



    return True

#清晰的显示箱体结构
def show_clearly(vis):
    global  color_list
    for kk_time in range(0,len(items0)):
        if (kk_time ==0):
            # 创建和添加坐标轴
            coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=3.0, origin=[0, 0, 0])
            container_size = [box_w, box_l, box_h, 0, 0, 0, 0]
            container, container_line_set = create_container(container_size)
            vis.add_geometry(coord_frame)
            vis.add_geometry(container)
            vis.add_geometry(container_line_set)
            opt = vis.get_render_option()
            opt.background_color = np.asarray([1, 1, 1])

            ctraa = vis.get_view_control()
            ctraa.set_lookat(np.array([box_w/2, box_l/2, box_h/2]))
            ctraa.set_zoom(1.2)
            ctraa.set_up((0, 0, 1))  # set the positive direction of the x-axis as the up direction
            ctraa.set_front((1, 0, 0))  # set the positive direction of the x-axis toward you


        ctr_before = vis.get_view_control()
        param_before = ctr_before.convert_to_pinhole_camera_parameters()


        if (kk_time < len(items0)):
        #    print(" kk_time=  ",kk_time,"  len(items0)=  ",len(items0)," items1[k_time]=  ",items0[kk_time])
            # 更新Cube和Line Set数据
            colors = (color_list[kk_time][0], color_list[kk_time][1], color_list[kk_time][2])
            cube1, line_set1 = create_cube1(items0[kk_time],colors)

            vis.add_geometry(cube1)
            vis.add_geometry(line_set1)
            vis.update_geometry(cube1)
            vis.update_geometry(line_set1)

            ctr_after = vis.get_view_control()
            ctr_after.convert_from_pinhole_camera_parameters(param_before)
            kk_time+=1
        else:
            return False
    return True


def on_button_clicked():
    print("Button clicked!")
class my_MainWindow(QMainWindow):
    def __init__(self,parent=None):
        super(my_MainWindow, self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(visible=False)  # visible=False窗口不显示，避免启动时一闪而过

        self.winid = win32gui.FindWindow('GLFW30', None)
        self.sub_window = QWindow.fromWinId(self.winid)
        self.displayer = QWidget.createWindowContainer(self.sub_window)
        self.ui.gridLayout.addWidget(self.displayer)

        # 创建128个单元格的表格
        #self.ui.tableWidget=QTableWidget()
        self.ui.tableWidget.setEditTriggers(QTableWidget.NoEditTriggers)  # 禁止修改
        self.ui.tableWidget.verticalHeader().setDefaultSectionSize(50)  # 行高50
        self.ui.tableWidget.horizontalHeader().setDefaultSectionSize(110)  # 列宽110
        self.ui.tableWidget.verticalHeader().setVisible(False)  # 不显示行标题
        self.ui.tableWidget.horizontalHeader().setVisible(False)  # 不显示列标题
        self.ui.tableWidget.setColumnCount(5)
        self.ui.tableWidget.setRowCount(len(items0)+1)

        # 说明行
        text1 = "箱子序列号"
        text2 = "箱体尺寸"
        text3="箱体体积"
        userate_text = "整体利用率:"
        height_text = "重心高度:"

        M1 = QTableWidgetItem()
        M2 = QTableWidgetItem()
        M3 = QTableWidgetItem()
        M_userate_text = QTableWidgetItem()
        M_height_text = QTableWidgetItem()
        M1.setText(text1)
        M2.setText(text2)
        M3.setText(text3)
        M_userate_text.setText(userate_text)
        M_height_text.setText(height_text)
        self.ui.tableWidget.setItem(0, 0, M1)
        self.ui.tableWidget.setItem(0, 1, M2)
        self.ui.tableWidget.setItem(0, 2, M3)
        self.ui.tableWidget.setItem(0, 3, M_userate_text)
        self.ui.tableWidget.setItem(0, 4, M_height_text)
        self.ui.tableWidget.item(0, 0).setTextAlignment(Qt.AlignCenter)
        self.ui.tableWidget.item(0, 1).setTextAlignment(Qt.AlignCenter)
        self.ui.tableWidget.item(0, 2).setTextAlignment(Qt.AlignCenter)
        self.ui.tableWidget.item(0, 3).setTextAlignment(Qt.AlignCenter)
        self.ui.tableWidget.item(0, 4).setTextAlignment(Qt.AlignCenter)
        self.ui.tableWidget.item(0, 0).setBackground(QBrush(QColor(204, 204, 204)))  # 改变单元格颜色
        self.ui.tableWidget.item(0, 1).setBackground(QBrush(QColor(204, 204, 204)))  # 改变单元格颜色
        self.ui.tableWidget.item(0, 2).setBackground(QBrush(QColor(204, 204, 204)))  # 改变单元格颜色
        self.ui.tableWidget.item(0, 3).setBackground(QBrush(QColor(204, 204, 204)))  # 改变单元格颜色
        self.ui.tableWidget.item(0, 4).setBackground(QBrush(QColor(204, 204, 204)))  # 改变单元格颜色

        for i in range(len(items0)):
              tmp_i=i+1
              #标签列
              name = QTableWidgetItem()
              DI_Number = str(i)
              # 定义一个字符串 要添加会变化的字符
              name.setText(DI_Number)
              # 赋予到表格元素中
              self.ui.tableWidget.setItem(tmp_i, 0, name)
              # # 显示到N行N列
              self.ui.tableWidget.item(tmp_i, 0).setTextAlignment(Qt.AlignCenter)
              # # 设置文字居中
              self.ui.tableWidget.item(tmp_i, 0).setBackground(QBrush(QColor(color_list[i][0] * 255.0, color_list[i][1] * 255.0, color_list[i][2] * 255.0))) # 改变单元格颜色

              #尺寸列
              name2 = QTableWidgetItem()
              DI_Number2 = "("+str(items0[i][0])+","+str(items0[i][1])+","+str(items0[i][2])+")"
              name2.setText(DI_Number2)
              self.ui.tableWidget.setItem(tmp_i, 1, name2)
              self.ui.tableWidget.item(tmp_i, 1).setTextAlignment(Qt.AlignCenter)
              self.ui.tableWidget.item(tmp_i, 1).setBackground(QBrush(QColor(204,204, 204)))  # 改变单元格颜色

              #体积列
              name1 = QTableWidgetItem()
              DI_Number1 = str(items0[i][0]*items0[i][1]*items0[i][2])
              name1.setText(DI_Number1)
              self.ui.tableWidget.setItem(tmp_i, 2, name1)
              self.ui.tableWidget.item(tmp_i, 2).setTextAlignment(Qt.AlignCenter)
              self.ui.tableWidget.item(tmp_i, 2).setBackground(QBrush(QColor(204,204, 204)))  # 改变单元格颜色



        start_button = self.ui.Start_Button
        start_button.setEnabled(True)
        pause_button = self.ui.Pause_Button
        pause_button.setEnabled(False)
        before_button = self.ui.Before_Button
        before_button.setEnabled(False)
        next_button = self.ui.Next_Button
        next_button.setEnabled(False)
        # 将按钮点击事件连接到槽函数
        start_button.clicked.connect(self.start_raise)
        before_button.clicked.connect(self.before_raise)
        pause_button.clicked.connect(self.pause_raise)
        next_button.clicked.connect(self.next_raise)

        self.timer = QTimer()  # 初始化定时器
        self.timer.timeout.connect(self.draw_update)
        self.time_interval=1.0
        self.flag=0             #初始化状态



    def before_raise(self):
        self.draw_update_before()

    def next_raise(self):
        self.draw_update_next()
    def start_raise(self):
        self.flag=1
        self.time_interval = 0.5
        self.timer.start(20)
        self.ui.Pause_Button.setEnabled(True)
        self.ui.Start_Button.setEnabled(False)
        self.ui.Before_Button.setEnabled(False)
        self.ui.Next_Button.setEnabled(False)

    def pause_raise(self):
        self.flag = 2
        self.time_interval = 1e6
        #self.timer.stop()
        self.ui.Start_Button.setEnabled(True)
        self.ui.Pause_Button.setEnabled(False)
        self.ui.Before_Button.setEnabled(True)
        self.ui.Next_Button.setEnabled(True)


    def draw_update_before(self):
        global use_rate, weight_center_height, balance_boxs, unbalanced_boxs,k_time
        k_time-=2
        update_callback_no_interval(self.vis,1)
        self.vis.poll_events()
        self.vis.update_renderer()
        if k_time==-1:
            self.ui.Before_Button.setEnabled(False)


        # 利用率
        text_userate = f'{use_rate:.5f}'
        M_userate = QTableWidgetItem()
        M_userate.setText(text_userate)
        self.ui.tableWidget.setItem(1, 3, M_userate)
        self.ui.tableWidget.item(1, 3).setTextAlignment(Qt.AlignCenter)
        self.ui.tableWidget.item(1, 3).setBackground(QBrush(QColor(204, 204, 204)))  # 改变单元格颜色

        # 重心高度
        text_weight_center_height = f'{weight_center_height:.5f}'
        M_weight_center_height = QTableWidgetItem()
        M_weight_center_height.setText(text_weight_center_height)
        self.ui.tableWidget.setItem(1, 4, M_weight_center_height)
        self.ui.tableWidget.item(1, 4).setTextAlignment(Qt.AlignCenter)
        self.ui.tableWidget.item(1, 4).setBackground(QBrush(QColor(204, 204, 204)))  # 改变单元格颜色

        if k_time==len(items0):
            self.ui.Next_Button.setEnabled(False)
        else:
            self.ui.Next_Button.setEnabled(True)

    def draw_update_next(self):
        global use_rate, weight_center_height, balance_boxs, unbalanced_boxs
        update_callback_no_interval(self.vis)
        self.vis.poll_events()
        self.vis.update_renderer()

        # 利用率
        text_userate = f'{use_rate:.5f}'
        M_userate = QTableWidgetItem()
        M_userate.setText(text_userate)
        self.ui.tableWidget.setItem(1, 3, M_userate)
        self.ui.tableWidget.item(1, 3).setTextAlignment(Qt.AlignCenter)
        self.ui.tableWidget.item(1, 3).setBackground(QBrush(QColor(204, 204, 204)))  # 改变单元格颜色

        # 重心高度
        text_weight_center_height = f'{weight_center_height:.5f}'
        M_weight_center_height = QTableWidgetItem()
        M_weight_center_height.setText(text_weight_center_height)
        self.ui.tableWidget.setItem(1, 4, M_weight_center_height)
        self.ui.tableWidget.item(1, 4).setTextAlignment(Qt.AlignCenter)
        self.ui.tableWidget.item(1, 4).setBackground(QBrush(QColor(204, 204, 204)))  # 改变单元格颜色


        if k_time==len(items0):
            self.ui.Next_Button.setEnabled(False)
        else:
            self.ui.Next_Button.setEnabled(True)

    def draw_update(self):
        global use_rate,weight_center_height,balance_boxs,unbalanced_boxs
        update_callback(self.vis,self.time_interval)
        self.vis.poll_events()
        self.vis.update_renderer()

        if k_time==len(items0):
            self.ui.Next_Button.setEnabled(False)


        # for i in range(len(items0)):
        #     if i in balance_boxs:
        #         DI_Number1 = "稳定"
        #         color_S=QColor(0, 255, 0)
        #     elif i in unbalanced_boxs:
        #         DI_Number1 = "不稳定"
        #         color_S=QColor(255,0 , 0)
        #     else:
        #         DI_Number1=""
        #         color_S=QColor(204, 204, 204)
        #
        #     tmp_i=i+1
        #     # 稳定度
        #     name1 = QTableWidgetItem()
        #
        #     name1.setText(DI_Number1)
        #     self.ui.tableWidget.setItem(tmp_i, 1, name1)
        #     self.ui.tableWidget.item(tmp_i, 1).setTextAlignment(Qt.AlignCenter)
        #     self.ui.tableWidget.item(tmp_i, 1).setBackground(QBrush(color_S))  # 改变单元格颜色

        #利用率
        text_userate = f'{use_rate:.5f}'
        M_userate = QTableWidgetItem()
        M_userate.setText(text_userate)
        self.ui.tableWidget.setItem(1 ,3, M_userate)
        self.ui.tableWidget.item(1, 3).setTextAlignment(Qt.AlignCenter)
        self.ui.tableWidget.item(1, 3).setBackground(QBrush(QColor(204, 204, 204)))  # 改变单元格颜色

        #重心高度
        text_weight_center_height = f'{weight_center_height:.5f}'
        M_weight_center_height = QTableWidgetItem()
        M_weight_center_height.setText(text_weight_center_height)
        self.ui.tableWidget.setItem(1, 4, M_weight_center_height)
        self.ui.tableWidget.item(1, 4).setTextAlignment(Qt.AlignCenter)
        self.ui.tableWidget.item(1, 4).setBackground(QBrush(QColor(204, 204, 204)))  # 改变单元格颜色

    def draw_weight(self):
        self.vis.clear_geometries()
        show_clearly(self.vis)
        centers = [calculate_center(item) for item in items0]
        ball_color=(1,0,0)
        for center in centers:
            my_weight_vis=create_WeightPoint(center[0],center[1],center[2],ball_color,0.05)
            # 显示点云
            self.vis.add_geometry(my_weight_vis)
            self.vis.update_geometry(my_weight_vis)

        P1 = calculate_centroid(items0, 1)
        my_weight_vis2 = create_WeightPoint(P1[0], P1[1], P1[2], (0,0,1),0.1)
        print("总体重心=",P1)

        #coord_weight = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[P1[0], P1[1], P1[2]])
        self.vis.add_geometry(my_weight_vis2)
        self.vis.update_geometry(my_weight_vis2)

        ctr_after = self.vis.get_view_control()
        ctr_after.convert_from_pinhole_camera_parameters(param_before)

        self.vis.poll_events()
        self.vis.update_renderer()




class Cuboid:
    def __init__(self,  length, width, height, x=0, y=0, z=0):
        self.length = length
        self.width = width
        self.height = height
        self.pos = [x, y, z]
        self.body_id = None


    def create(self):
        half_extents = [self.length / 2, self.width / 2, self.height / 2]
        self.body_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
        p.createMultiBody(baseMass=1, baseCollisionShapeIndex=self.body_id, basePosition=self.pos)



if __name__ == "__main__":
   #  p.connect(p.GUI)  # 必须先连接服务器
   #  p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
   #  p.setAdditionalSearchPath(pybullet_data.getDataPath())
   #  p.setRealTimeSimulation(1)
   #  # 创建地面平面
   # # p.loadURDF("plane.urdf", useMaximalCoordinates=True)
   #  floor_shape = p.createCollisionShape(p.GEOM_PLANE,halfExtents=[box_l,box_w,1])
   #  floor_body = p.createMultiBody(baseCollisionShapeIndex=floor_shape, basePosition=[0, 0, 0])
   #  # 设置视角
   #  camera_dist = 30
   #  camera_yaw = 40
   #  camera_pitch = -30
   #  camera_target_pos = [0, 0, 10]
   #  p.resetDebugVisualizerCamera(camera_dist, camera_yaw, camera_pitch, camera_target_pos)
   #  times1=0
   #  cuboid_t=[]
   #  for itemsPP in items1:
   #      if times1!=5:
   #          p.setGravity(0, 0, gravZ=-9.8)  # 未指明则默认第一个物理引擎（本例中GUI）
   #          length = itemsPP[0]
   #          width = itemsPP[1]
   #          height = itemsPP[2]
   #          x = itemsPP[3]
   #          y = itemsPP[4]
   #          z = itemsPP[5]
   #          # 创建长方体的碰撞形状
   #          half_extents = [length/2,width/2,height/2]  # 长方体 x, y, z 方向上的半边长
   #          collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
   #          # 创建长方体的初始位置和姿态
   #          position = [x+length/2, y+width/2, z+height/2+0.5]  # 初始位置 (x, y, z)
   #          p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
   #          body_id = p.createMultiBody(baseMass=length*width*height,baseCollisionShapeIndex=collision_shape_id, basePosition=position)
   #          # 创建结束，重新开启渲染
   #          p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
   #          cuboid_t.append(body_id)
   #          time.sleep(1)
   #      times1+=1
   #  time.sleep(1000)
   #





    tpo_center = calculate_centroid(items1, 1)
    print("总体重心=", tpo_center)
    items0=items1

    #计算colorlist
    N = len(items0)
    color_list = distinctipy.get_colors(N)

    for times in range(0,N):
        colors = (color_list[times][0], color_list[times][1], color_list[times][2])
        cube1, line_set1 = create_cube(items0[times], colors, times)
        cubes_Total.append(cube1)
        lines_Total.append(line_set1)


    #color_list = ncolors(N)

    app = QApplication(sys.argv)
    window = my_MainWindow()
    window.show()
    sys.exit(app.exec_())
