# This is a sample Python script.

import sys
sys.path.append("E:/HN University/机器人竞赛/代码/RM_code/HN Vedio")
import detect_target
import PredictDeltaW
import Fitting
import predict_point
import cv2

if __name__ == 'main':
    vedio = cv2.VideoCapture(r'E:\HN University\机器人竞赛\视频\2021-05-12 22_35.avi')