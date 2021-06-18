# -*- coding: utf-8 -*-
import time
import sys
sys.path.append("E:/HN University/机器人竞赛/代码/RM_code/HN Vedio")
import open_CSV

if __name__ == '__main__':
    file_path = 'E:\HN University\机器人竞赛\代码\data_2021_05_25_15_47_21.csv'
    date = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())
    f = open("E:\HN University\机器人竞赛\代码\Change_" + date + ".csv", "a+")
    init_list = open_CSV.open_path(file_path)
    for i in range(0,len(init_list))
