import open3d as o3d
import numpy as np

print("->正在加载点云... ")
pcd = o3d.io.read_point_cloud("bunny.pcd")
print(pcd)

print("->正在可视化点云")
o3d.visualization.draw_geometries([pcd])
