# -*- coding: utf-8 -*-

#该代码采用4个点进行预测，没有进行滤波拟合


import sys
sys.path.append("E:/HN University/机器人竞赛/代码/RM_code/HN Vedio")
import open_CSV
import time
import numpy as np
import math
import sympy as sy

def w_speed(w,time):
    w_speed = w/time
    return w_speed

def w_accelerate(w1,w2,time):
    accelerate = (w2-w1)/time
    return accelerate

def time_list(init_list): #obtain time in init list
    t_list =[]
    for i in range(0,len(init_list)):
        t_list.append(init_list[i][0])
    return  t_list

def alter_list(init_list,a,b):#obain a,b column
    point_list =[]
    for i in  range(0,len(init_list)):
        L_list = []
        L_list.append(init_list[i][a])
        L_list.append(init_list[i][b])
        point_list.append(L_list)
    return point_list

def angle(v1, v2):
  dx1 = v1[2] - v1[0]
  dy1 = v1[3] - v1[1]
  dx2 = v2[2] - v2[0]
  dy2 = v2[3] - v2[1]
  angle1 = math.atan2(dy1, dx1)
  angle2 = math.atan2(dy2, dx2)
  if angle1*angle2 >= 0:
    included_angle = abs(angle1-angle2)
  else:
    included_angle = abs(angle1) + abs(angle2)
    if included_angle > np.pi:
      included_angle = 2*np.pi - included_angle
  return included_angle

def init_t(w, w_accelerate):
    # 角速度与角加速度
    if w_accelerate == 0: #加速度=0时，不能做判断
        speed_time = 2*sy.pi
    if w >= 1.305 and w_accelerate >= 0:
        speed_time = sy.asin((w - 1.305) / 0.785) / 1.884
    elif w >= 1.305 and  w_accelerate < 0:
        speed_time = (sy.pi - sy.asin((w - 1.305) / 0.785)) / 1.884
    elif w < 1.305 and w_accelerate >= 0:
        speed_time = (2 * sy.pi + sy.asin((w - 1.305) / 0.785)) / 1.884
    elif w < 1.305 and w_accelerate < 0:
        speed_time = (sy.pi - sy.asin((w - 1.305) / 0.785)) / 1.884
    return speed_time

if __name__ == '__main__':
    file_path = 'E:\HN University\机器人竞赛\代码\RM_code\HN vedio\DATA_2021-05-12 22_35.csv' #原始点位
    delta_time = 1/100
    predict_time = 2
    date = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())
    f = open("E:\HN University\机器人竞赛\代码\RM_code\HN vedio\P_" + date + ".csv", "a+")
    init_list = open_CSV.open_path(file_path) #列表转换
    t_list = time_list(init_list)#时间列
    center_list = np.array(alter_list(init_list, 1, 2))#中心列
    target_list = alter_list(init_list, 3, 4)#大幅列

    DwList = [] #获取中心点坐标
    TempList = []
    for i in range(0, (len(init_list) - 30)): #30帧得到1个角度及0.3s
        TempList.append(target_list[i])
        TempList.append(target_list[i + 30])
        DwList.append(TempList)
        TempList = []
    Center = np.mean(center_list,axis=0) #中心平均值

    w= [] #获取角度差/30帧
    for i in range(len(DwList)):
        v1 = [Center[0], Center[1], DwList[i][0][0], DwList[i][0][1]]
        v2 = [Center[0], Center[1], DwList[i][1][0], DwList[i][1][1]]
        Angle = angle(v1, v2)
        w.append(Angle)

    w_sp = [] #获得角速度
    for i in range(0,len(w)):
        w_velocity=w_speed(w[i],0.3) #以0.3s为间隔
        w_sp.append(w_velocity)

    w_ac = [] #获得角加速度
    for i in range(0,len(w_sp)-1):
        accelerate = (w_sp[i+1]-w_sp[i])/delta_time
        w_ac.append(accelerate)

    w_ac_t_speed=[] #角加速度时刻的角速度
    for i in range(0,len(w_sp)-1):
        speed = (w_sp[i]+w_sp[i+1])/2
        w_ac_t_speed.append(speed)

    t = []
    for i in range(0,len(w_ac)):
        t = init_t(w_ac_t_speed[i],w[i])
        f.write(str(t)+'\n')
    f.close()
    print('done!!')

