# -*- coding: utf-8 -*-
"""
Created on Tue Apr 27 22:22:43 2021

@author: Makunatata
"""


import time
import math
import csv
import numpy as np

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

if __name__ == '__main__':
    file_path = 'E:\HN University\机器人竞赛\代码\RM_code\HN vedio\data_2021_05_25_09_38_21.csv'
    date = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())
    f = open("E:\HN University\机器人竞赛\代码\RM_code\HN vedio\DW_data_" + date + ".csv", "a+")
    #间隔1.6s读取角度插值，160frame读取一次差值
    init_list=open_path(file_path)
    DwList=[]
    TempList =[]
    for i in range(0,(len(init_list)-50)):
        TempList.append(init_list[i])
        TempList.append(init_list[i+50])
        DwList.append(TempList)
        TempList=[]
    CenterList = []
    for i in range(0,(len(init_list)-50)):
        a = 0
        b = 0
        for j in range(0,50):
            a = a + init_list[i+j][0]
            b = b + init_list[i+j][1]
        center = [a/50,b/50]
        CenterList.append(center)
    for i in range(len(DwList)):
        v1 = [CenterList[i][0],CenterList[i][1],DwList[i][0][2],DwList[i][0][3]]
        v2 = [CenterList[i][0],CenterList[i][1],DwList[i][1][2],DwList[i][1][3]]
        Angle = angle(v1,v2)
        f.write(str(Angle) + '\n')
    f.close()
    print('Done')
