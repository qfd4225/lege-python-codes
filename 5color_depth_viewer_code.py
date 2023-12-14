from pyorbbecsdk import Pipeline, FrameSet
from pyorbbecsdk import Config
from pyorbbecsdk import OBSensorType, OBFormat
from pyorbbecsdk import OBError
from pyorbbecsdk import VideoStreamProfile
import cv2
import numpy as np
from utils import frame_to_bgr_image
import os

from pyorbbecsdk import *
ESC_KEY = 27


pattern_size = (6, 4) 

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

    pipeline.start(config)
    temp = 1
    while True:
        try:
            frames: FrameSet = pipeline.wait_for_frames(100)
            if frames is None:
                continue
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if color_frame is None:
                continue
            # covert to RGB format
            color_image = frame_to_bgr_image(color_frame)
            
            color_copy = color_image.copy()
            
            # gray = cv2.cvtColor(color_copy, cv2.COLOR_BGR2GRAY)
            # arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
            # arucoParams = cv2.aruco.DetectorParameters()
            #(corners, ids, rejected) = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)
            # if len(corners) == 4:
            #     point_id_ = 0
            #     for (markerCorner, markerID) in zip(corners, ids):
            #         corners_ = markerCorner.reshape((4, 2))
            #         (topLeft, topRight, bottomRight, bottomLeft) = corners_
            #         topRight = (int(topRight[0]), int(topRight[1]))
            #         bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            #         bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            #         topLeft = (int(topLeft[0]), int(topLeft[1]))
            #         cv2.line(color_copy, topLeft, topRight, (0, 255, 0), 2)
            #         cv2.line(color_copy, topRight, bottomRight, (0, 255, 0), 2)
            #         cv2.line(color_copy, bottomRight, bottomLeft, (0, 255, 0), 2)
            #         cv2.line(color_copy, bottomLeft, topLeft, (0, 255, 0), 2)
            #         cv2.putText(color_copy, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            #         for corner_pt in markerCorner.squeeze():
            #             corner_pt = corner_pt.squeeze()
            #             cv2.putText(color_copy, str(point_id_), (int(corner_pt[0]), int(corner_pt[1])),cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255, 0, 0), 1)
            #             point_id_ += 1
                    
            # cv2.imshow("Chessboard Detection-color_copy", color_copy)
            
            
            if color_image is None:
                print("failed to convert frame to image")
                continue
            #cv2.imshow("Color Viewer", color_image)
            cv2.imshow("Chessboard Detection-color_copy", color_copy)
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
                save_color_frame(color_frame, temp)
                save_depth_frame(depth_frame, temp)
                temp += 1
            elif key == ord('t'):
                save_color_frame_t(color_frame, temp)
                save_depth_frame_t(depth_frame, temp)
        except KeyboardInterrupt:
            break
    pipeline.stop()


if __name__ == "__main__":
    main()
