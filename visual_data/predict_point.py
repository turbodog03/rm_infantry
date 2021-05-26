# -*- coding: utf-8 -*-
import numpy as np
import csv
import time
import sympy as sy
import math

def open_path(filename): #打开路径，数值列表
    f = open(filename,'r')
    csvreader = csv.reader(f)
    str_list = list(csvreader)
    N_list = list_Transfer(str_list)
    return N_list

def list_Transfer(str_list): #字符列表，转化为数字列表
    N_list = []
    for i in range(len(str_list)):
        S_list = []
        for j in range(len(str_list[i])):
            element = float(str_list[i][j])
            S_list.append(element)
        N_list.append(S_list)
    return N_list

def func(t,a):
    w = 1.305 * t - 0.416666666666667 * sy.cos(1.884 *t + a )
    return w

def integral(a,t1,t2):
    w = func(t2,a) - func(t1,a)
    return w

def angle(v1, v2):
    dx1 = v1[2] - v1[0]
    dy1 = v1[3] - v1[1]
    dx2 = v2[2] - v2[0]
    dy2 = v2[3] - v2[1]
    angle1 = math.atan2(dy1, dx1)
    angle2 = math.atan2(dy2, dx2)
    if angle1 * angle2 >= 0:
        included_angle = abs(angle1 - angle2)
    else:
        included_angle = abs(angle1) + abs(angle2)
        if included_angle > np.pi:
            included_angle = 2 * np.pi - included_angle
    return included_angle

def distence(p1,p2):
    length = math.hypot(p1,p2)
    return length

def flatten(input_list): #python层list解嵌套
    output_list = []
    while True:
        if input_list == []:
            break
        for index, i in enumerate(input_list):
            if type(i)== list:
                input_list = i + input_list[index+1:]
                break
            else:
                output_list.append(i)
                input_list.pop(index)
                break
    return output_list

def square(p1,p2):
    vector_l=[p1[0]-p2[0],p2[1]-p1[1]]
    vector = np.array(vector_l)
    distence=math.hypot(vector[0],vector[1])
    return distence
    

if __name__ == '__main__':
    file_path_1 = 'E:\HN University\机器人竞赛\代码\RM_code\HN vedio\LSDW_data_2021_05_25_11_29_15.csv'
    file_path_2 = 'E:\HN University\机器人竞赛\代码\RM_code\HN vedio\data_2021_05_25_09_38_21.csv'
    t = 2  # 预测时间间隔
    date = time.strftime('%Y_%m_%d_%H_%M_%S', time.localtime())
    f = open("E:\HN University\机器人竞赛\代码\RM_code\HN vedio\YCDW_data_" + date + ".csv", "a+")
    w_phase = open_path(file_path_1)
    w_phase = flatten(w_phase)
    v_phase = []
    a = 0.471 #角度与角速度函数的相位差 1.884*1/2/2
    for i in range (0,len(w_phase)):
        v = w_phase[i] - a
        v_phase.append(v)
    init_position = open_path(file_path_2)#视频读取到的位置
    center = []
    target = []
    for i in range(0,len(init_position)):
        T_center = []
        T_center.append(init_position[i][0])
        T_center.append(init_position[i][1])
        center.append(T_center)
        T_target = []
        T_target.append(init_position[i][2])
        T_target.append(init_position[i][3])
        target.append(T_target)
    #中心位置坐标（平均数）
    center_position = np.array(center)
    center_position = np.mean(center,axis=0)
    #初始角度，目标与x轴正向夹角
    I_angle = []
    for i in range(0,len(init_position)-200):
        v1 = [center_position[0], center_position[1], target[i][0], target[i][1]]
        v2 = [center_position[0], center_position[1], center_position[0]+2, center_position[1]]
        if target[i][1] > center[i][1]:
            init_angle = angle(v1, v2)
        else:
            init_angle = 2 * np.pi - angle(v1, v2)
        I_angle.append(init_angle)
    #预测位置与初始位置的夹角
    integral_angel = []
    for i in range(0,len(init_position)-200):
        integral_A = integral(v_phase[i],0,t)
        integral_angel.append(integral_A)
    #预测位置与x轴正向的夹角
    A = []
    for i in range(0,len(init_position)-200):
        A_L = -integral_angel[i]+I_angle[i]
        A.append(A_L)
    #半径像素点个数
    length =[]
    for i in range(0,len(init_position)-200):
        dis =square(center[i],target[i])
        length.append(dis)
    #预测点坐标
    New_point=[]
    for i in range(0,len(init_position)-200):
        float_a =float(A[i])
        p_x = center_position[0]+length[i]*np.cos(float_a)
        p_y = center_position[1]+length[i]*np.sin(float_a)
        position = []
        position.append(p_x)
        position.append(p_y)
        New_point.append(position)
        f.write(str(position[0])+','+str(position[1])+'\n')
    f.close()
    print('done')