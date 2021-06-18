# -*- coding: utf-8 -*-
import open_CSV
import sympy as sy

# def CalculateInitTime(t,angle):
#     #和差化积,计算求解init_T
#     TemporaryVariable = sy.asin(0.942/0.875/sy.sin(0.942t)*(angle - 1.305*t))
#     if TemporaryVariable >= 0:
#         init_t = (TemporaryVariable - 1.884*t)/2/1.884
#     else:
#         init_t = ((sy.pi-TemporaryVariable)-1.884*t)/2/1.884

def Instant_W(t,angle):
    #angle为间隔t时间的角度
    w = angle/t
    return w

def Instant_accelerated_W(w1,w2,t):
    #w1，w2为间隔检测的角速度
    a = (w2 - w1)/t
    return a

def Cal_Init_T(angle,t,w,a):
    #反解积分三角函数——时间间隔积分，求出初始t值，即相位
    if w-1.305 > 0 and a > 0:
        init_t = (sy.asin(0.942 / 0.875 / sy.sin(0.942 * t) * (angle - 1.305 * t))-0.942 * t)/ 1.884
    elif w-1.305 > 0 and a < 0:
        init_t = (sy.pi - sy.asin(0.942 / 0.875 / sy.sin(0.942 * t) * (angle - 1.305 * t)) - 0.942 * t) / 1.884
    elif w-1.305 < 0 and a < 0:
        init_t = (sy.pi - sy.asin(0.942 / 0.875 / sy.sin(0.942 * t) * (angle - 1.305 * t)) - 0.942 * t) / 1.884
    elif w -1.305 <0 and a < 0:
        init_t = (2 * sy.pi - sy.asin(0.942 / 0.875 / sy.sin(0.942 * t) * (angle - 1.305 * t)) - 0.942 * t) / 1.884
    return init_t

def Predict_location(init_time,center,Predict_Time,):
    #初始时间init_time,识别圆心center，预测时间间隔t，









improt open_CSV
if __name__ == '__main__':
    file_path = 'E:\HN University\机器人竞赛\代码\RM_code\HN vedio\data_2021_04_27_22_24_10.csv'
    date = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())
    f = open("E:\HN University\机器人竞赛\代码\RM_code\HN vedio\DW_data_" + date + ".csv", "a+")
    init_list = open_CSV.open_path(file_path) #τ积分角度序列
    delta = 2/3
    Predict_Time = 1

