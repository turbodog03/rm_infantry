# -*- coding: utf-8 -*-
import csv
import time

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

if __name__== '__main__':
    file_path = 'E:\HN University\机器人竞赛\代码\RM_code\HN vedio\data_2021_04_23_14_46_53.csv'
    N_list = open_path(file_path)
    print(type(N_list))
    date = time.strftime("%Y_%m_%d_%H_%M_%S",time.localtime())
    f = open("E:\HN University\机器人竞赛\data_"+date+".csv","w")
    for line in N_list:
        f.write(str(line[0])+','+str(line[1])+'\n')
    f.close()
    print('Done!')


