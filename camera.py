from machine import *

from smartcar import *
from seekfree import *
from display import *

import UI

import gc
import time


# 调用 TSL1401 模块获取 CCD 实例
# 参数是采集周期 调用多少次 capture 更新一次数据
# 默认参数为 1 调整这个参数相当于调整曝光时间倍数
ccd = TSL1401(9)



####################################平均值法#######################################
def erzhihua(data):
    data2zh=[]
    summ=0
    geshu=0
    
    for i in data:
        summ=summ+i
        geshu=geshu+1
    mid=summ/geshu
    if(mid>=200):
        mid=200
    elif(mid<=30):
        mid=30
        
    for i in range(len(data)):#tsl1401宽度为128，数据范围0~255
#         if (data[i]>=mid):#二值化参数
        if (data[i]>=mid):#二值化参数
            data2zh.append(0)#白
        else:
            data2zh.append(1)#黑
    data2zh[0]=1
    data2zh[1]=1
    data2zh[127]=1
    data2zh[126]=1
    return data2zh



####################################固定阈值法#######################################
def gudingyuzhi(data):
    data2zh=[]
    maxnum=0
    
    for i in data:
        if(i>=maxnum):
            maxnum=i
    
    for i in range(len(data)):#tsl1401宽度为128，数据范围0~255
        if (data[i]>=maxnum-20 and data[i]>=200):#固定阈值给的230
            data2zh.append(0)#白
        else:
            data2zh.append(1)#黑
    data2zh[0]=1
    data2zh[1]=1
    data2zh[127]=1
    data2zh[126]=1
    return data2zh


    

####################################采集ccd数据########################################
def cameraprocess():
    # 通过 capture 接口更新数据 但在这个例程中被 ticker 模块接管了
    # ccd.capture()
    # 通过 get 接口读取数据 参数 [0,1] 对应学习板上 CCD1/2 接口
    global ccd_data1
    global ccd_data2
    global ccd2zh_data1
    global ccd2zh_data2
    
    ccd_data1 = ccd.get(0)
    ccd_data2 = ccd.get(1)
    
    
#     ccd2zh_data1 = dajinfa(ccd_data1)
#     ccd2zh_data2 = dajinfa(ccd_data2)
    ccd2zh_data1 = gudingyuzhi(ccd_data1)
    ccd2zh_data2 = gudingyuzhi(ccd_data2)
#     ccd2zh_data1 = erzhihua(ccd_data1)
#     ccd2zh_data2 = erzhihua(ccd_data2)

#         time.sleep_ms(1000)
#         lcdline()

