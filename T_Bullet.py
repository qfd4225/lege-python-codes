#!/usr/bin/python3
import pybullet as p
import pybullet_data
from time import sleep


p.connect(p.GUI) # 必须先连接服务器
p.setGravity(0, 0, -9.8)


# 设置模型加载路径
datapath = pybullet_data.getDataPath()
p.setAdditionalSearchPath(datapath)

# 加载模型
robot_id = p.loadURDF("r2d2.urdf", [0, 0, 0.5])

# 在这停顿
sleep(1000)
