from machine import *


from smartcar import *
from seekfree import *
from display import *

import time
import gc

import ticker
import findline

encoder_l = encoder("D0", "D1", True)
encoder_r = encoder("D2", "D3",True)

speed_over = 0 #完赛平均时间

speed_avr = 0

SPEED_F=100.0 #速度获取频率

L_QD_UNIT=9300.0 #编码器一米的计数
R_QD_UNIT=4650.0


r_filter=[0,0,0,0]
l_filter=[0,0,0,0]
L_CarSpeed=0
R_CarSpeed=0
        
L_sum=0
R_sum=0

lap_length=0


def getspeed():
    global speed_over
    global L_CarSpeed
    global R_CarSpeed
    global speed_avr
    
    global R_sum
    global L_sum
    global Left_count
    global Right_count
    
    global lap_length
    
    global r_filter
    global l_filter
    
    Right_count = encoder_r.get()
    Left_count= -encoder_l.get()
    
    L_sum=L_sum+Left_count
    R_sum=R_sum+int(Right_count)*2
    
    r_filter[3]=Right_count
    for i in range(3):
        r_filter[i] = r_filter[i + 1]
    l_filter[3]=Left_count
    for i in range(3):
        l_filter[i] = l_filter[i + 1]
    
#     L_CarSpeed = L_CarSpeed*0.15 + 0.85*(0.4*l_filter[3]+0.3*l_filter[2]+0.2*l_filter[1]+0.1*l_filter[0]) * SPEED_F / L_QD_UNIT    #（计数值*计数频率/一米计数值）求出车速转换为M/S
#     R_CarSpeed = R_CarSpeed*0.15 + 0.85*(0.4*r_filter[3]+0.3*r_filter[2]+0.2*r_filter[1]+0.1*r_filter[0]) * SPEED_F / R_QD_UNIT    #求出车速转换为M/S

    L_CarSpeed = (L_CarSpeed*0.2 + 0.2*l_filter[3]+0.2*l_filter[2]+0.2*l_filter[1]+0.2*l_filter[0]) * SPEED_F / L_QD_UNIT    #（计数值*计数频率/一米计数值）求出车速转换为M/S
    R_CarSpeed = (R_CarSpeed*0.2 + 0.2*r_filter[3]+0.2*r_filter[2]+0.2*r_filter[1]+0.2*r_filter[0]) * SPEED_F / R_QD_UNIT    #求出车速转换为M/S
        
#     L_CarSpeed=Left_count* SPEED_F / L_QD_UNIT    #（计数值*计数频率/一米计数值）求出车速转换为M/S
#     R_CarSpeed=Right_count* SPEED_F / R_QD_UNIT

#     speed_avr= (L_CarSpeed+R_CarSpeed)/2
    
    lap_length=(L_sum+R_sum)/2/L_QD_UNIT
    
    if(L_sum>=100 and findline.stopflag==0):
        speed_over=lap_length/(ticker.time_count/10)
