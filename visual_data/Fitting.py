# -*- coding: utf-8 -*-
"""
Created on Fri May  7 15:47:38 2021

@author: Makunatata
"""

import time
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import numpy as np
import csv

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
# 定义需要拟合的函数
def func(x, a):
    return 0.6525 + 0.3781495 * np.sin(x * 1.884 + a)  #0.5s时间间隔的角度
    
if __name__ == '__main__':
    file_path = 'E:\HN University\机器人竞赛\代码\RM_code\HN vedio\DW_data_2021_05_25_11_21_38.csv'
    date = time.strftime('%Y_%m_%d_%H_%M_%S',time.localtime())
    f = open("E:\HN University\机器人竞赛\代码\RM_code\HN vedio\LSDW_data_" + date + ".csv", "a+")
    ydata = open_path(file_path)
    #obtain csv data as y
    num =150 # 单组拟合数据个数
    for i in range(len(ydata)-num):  #测试最小拟合数据
        xdata = np.linspace(0,num/100,num)
        y_1data = np.array(ydata[i:i+num])
        y_1data = np.reshape(y_1data,xdata.shape)
        a = 0
        #w = 1.884
        p0 = [a] 
        para, _ = curve_fit(func, xdata, y_1data, p0)
        f.write(str(para[0])+'\n')
    f.close()
    print('done')
        
        
    # #整体数据拟合
    # num = len(ydata)
    # xdata = np.linspace(0,num/100,num)
    # #depend on time series 1/150
    # plt.plot(xdata,ydata,'b-',label='data')
    
    # ydata = np.array(ydata) 
    # ydata = np.reshape(ydata,xdata.shape)
    # #alter shape
    # a = 1.305
    # b = 0.785
    # c = 0
    # w =1.884
    # p0 = [a,b,c,w] 
    # #set init para
    # para, _ = curve_fit(func, xdata, ydata, p0)
    # print(para)
    # y = [func(a,*para) for a in xdata]
    # plt.plot(xdata, y, 'g--')
    # plt.xlabel('x')
    # plt.ylabel('y')
    # plt.show()