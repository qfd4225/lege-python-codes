import cv2
import numpy as np
#img = cv2.imread('save_images_code/c0.png', cv2.IMREAD_COLOR)
pic_test = cv2.imread('./QwIKM.png', cv2.IMREAD_COLOR)


# 设置目标尺寸
width = 4000
height = 3000

# 缩放图像
resized_image = cv2.resize(pic_test, (width, height))

cv2.imshow("image", resized_image)
cv2.imwrite("paint_module.png", resized_image)
cv2.waitKey(0)