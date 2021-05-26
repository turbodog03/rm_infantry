# -*- coding: utf-8 -*-
import cv2
import time

# def pretreatment(frame):  #周围存在部分杂物的预处理
#     b, g, r = cv2.split(frame) #按帧读取通道
#     mid = cv2.subtract(r,b) #通道相减
#     ret, binary = cv2.threshold(mid, 100, 255, cv2.THRESH_BINARY) #ret为bool，binary为三维矩阵 二值化处理
#     element_1 = cv2.getStructuringElement(cv2.MORPH_RECT,(2,2)) #卷积矩阵
#     eroded = cv2.erode(binary, element_1, iterations = 1)
#     element_2 = cv2.getStructuringElement(cv2.MORPH_RECT,(7,7)) #卷积矩阵
#     dilated = cv2.dilate(eroded,element_2,iterations = 1)
#     kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(7,7))
#     fin = cv2.morphologyEx(dilated ,cv2.MORPH_CLOSE,kernel,iterations = 3) #闭空间提取
#     return fin

def pretreatment(frame):  #HN自测预处理
    b, g, r = cv2.split(frame) #按帧读取通道
    mid = cv2.add(r,b) #通道相减
    ret, binary = cv2.threshold(mid, 100, 255, cv2.THRESH_BINARY) #ret为bool，binary为三维矩阵 二值化处理
    element_1 = cv2.getStructuringElement(cv2.MORPH_RECT,(2,2)) #卷积矩阵
    eroded = cv2.erode(binary, element_1, iterations = 1)
    element_2 = cv2.getStructuringElement(cv2.MORPH_RECT,(7,7)) #卷积矩阵
    dilated = cv2.dilate(eroded,element_2,iterations = 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(7,7))
    fin = cv2.morphologyEx(dilated ,cv2.MORPH_CLOSE,kernel,iterations = 3) #闭空间提取
    return fin

def detect_target(farme):
    img = pretreatment(frame)
    contours, hier = cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE) #contours-ndarray轮廓信息；hier-ndarray层级。矩形外框提取
    area = []
    C = []
    for i in range(len(contours)):
        area.append(cv2.contourArea(contours[i])) #计算最外层面积
        rect = cv2.minAreaRect(contours[i]) #（最小外接矩形的中心（x，y），（宽度，高度），旋转角度）
        #box  = cv2.boxPoints(rect) 四个顶点坐标
        C.append(rect) #找到最小外接矩形
    number = area.index(max(area)) #提取面积比最大
    (x,y),(w,h),r = cv2.minAreaRect(contours[number]) #目标矩形
    point = [float(x),float(y)]
    return point

def center_point(frame):
    #视频中圆心的坐标(中心点)
    img = pretreatment(frame)
    contours, hier = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    area = []
    R = []
    proportion = []
    for i in range(len(contours)):
        area.append(cv2.contourArea(contours[i]))
        R.append(cv2.minEnclosingCircle(contours[i]))
        proportion.append(area[i]/(R[i][1]*R[i][1]))
    number = proportion.index(max(proportion))
    (x,y),radius = cv2.minEnclosingCircle(contours[number])
    point = [float(x),float(y)]
    return point


if __name__ == '__main__':
    vedio = cv2.VideoCapture(r"E:\HN University\机器人竞赛\视频\2021-05-12 22_35.avi")
    #cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
    date = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())
    f = open("E:\HN University\机器人竞赛\代码\RM_code\HN vedio\data_" + date + ".csv", "a+")
    #frameNum = 0
    while(vedio.isOpened()):
        ret, frame = vedio.read()
        point = detect_target(frame)
        center = center_point(frame)
        f.write(str(center[0])+','+str(center[1])+','+str(point[0]) + "," + str(point[1]) + '\n')
        S_point = cv2.circle(frame, (int(point[0]),int(point[1])),radius=5,color=(255,0,0),thickness=-1)
        C_point = cv2.circle(frame, (int(center[0]),int(center[1])),radius=5,color=(0,255,0),thickness=-1)
        cv2.imshow('frame',S_point)
        cv2.imshow('frame',C_point)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    vedio.release()
    f.close()
    cv2.destroyAllWindows()

