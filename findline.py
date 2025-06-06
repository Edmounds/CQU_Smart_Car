from machine import *

from smartcar import *
from seekfree import *
from display import *

import encode
import ticker
import UI
import camera
import motor
import gyroscope

import gc
import time

leftline=0
rightline=127
midline1=63

midline=63
        
lasterror=0
err=0
err2=0
        
leftline2=0
rightline2=127
midline2=63
        
leftroundstate=0        
rightroundstate=0
straight=0
crossstate=0
xiepostate=0

#turn_angle_cross=0
turn_angle_right=0
turn_angle_left=0

stopflag=0

speed_state=0

####################################从中间向左右巡线#######################################

def erzhihua_findline():
    global leftline
    global rightline
    global midline1
    
    global midline
        
    global leftline2
    global rightline2
    global midline2
    
    global stopflag
    global xiepostate
    
    for i in range(midline1,-1,-1):
        if(camera.ccd2zh_data1[i]!=camera.ccd2zh_data1[i-1]):
            leftline=i
            break
    for i in range(midline1,127,+1):
        if(camera.ccd2zh_data1[i]!=camera.ccd2zh_data1[i+1]):
            rightline=i
            break
    midline1=(leftline+rightline)/2
    midline1=int(midline1)
        
    for i in range(midline2,0,-1):
        if(camera.ccd2zh_data2[i]!=camera.ccd2zh_data2[i-1]):
            leftline2=i
            break
    for i in range(midline2,127,+1):
        if(camera.ccd2zh_data2[i]!=camera.ccd2zh_data2[i+1]):
            rightline2=i
            break
    midline2=(leftline2+rightline2)/2
    midline2=int(midline2)
        
    if(camera.ccd2zh_data1[midline1]==1):
        if(camera.ccd2zh_data2[midline2]==0 and camera.ccd2zh_data1[midline1]==0):            
            midline1=midline2
        elif(camera.ccd2zh_data1[63]==0):
            midline1=63
    if(camera.ccd2zh_data2[midline2]==1):
        if(camera.ccd2zh_data2[midline2]==0 and camera.ccd2zh_data1[midline1]==0):
            midline2=midline1
        elif(camera.ccd2zh_data2[63]==0): 
            midline2=63
        
        
    if(camera.ccd2zh_data1[int(midline1*0.7+midline2*0.3)]==0 and camera.ccd2zh_data1[midline1]==0 and camera.ccd2zh_data2[midline2]==0):
        midline=int(midline1*0.6+midline2*0.4)
    else:
        midline=midline1
        
    
    if(encode.lap_length>=1 and stopflag==0 and xiepostate==0):
        Emergy_stop()
    
    
#######################################

                   
                   
####################################停车##############################################
overcount1=0
def Emergy_stop():
    global leftline
    global rightline
    global midline
    global leftline2
    global rightline2
    global midline2
    
    max1=0
#     maxcount1=0
    
    max2=0
#     maxcount2=0
#     global overcount1
    overcount1=0
#     overcount2=0
    global stopflag
    
    
#————————————————————常规出赛道急停——————————————————————————————————————————
    for i in camera.ccd_data1:
        if(i>=max1):
            max1=i
#             maxcount1=0
#         elif(max1==i):
#             maxcount1=maxcount1+1
            
#     for i in camera.ccd_data2:
#         if(i>=max2):
#             max2=i
#             maxcount2=0
#         elif(max2==i):
#             maxcount2=maxcount2+1
            
    if(max1<=240):
        stopflag=1
#——————————————————————————————————————————————————————————————
            
            
#————————————————————完赛急停——————————————————————————————————
    for i in range(23,104,+1):
        if(camera.ccd2zh_data1[i]!=camera.ccd2zh_data1[i+1]):
            overcount1=overcount1+1
#         if(camera.ccd2zh_data2[i]!=camera.ccd2zh_data2[i+1]):
#             overcount2=overcount2+1
    
    if(overcount1>=6 and rightline2-leftline2<=50):
        stopflag=2
            
            
#——————————————————————————————————————————————————————————————            
            
            
            
#######################################                       


        
####################################计算赛道偏差#######################################

def caculate_err():
    global lasterror
    global err
    global err2
    
    lasterror=err
    err=midline-63
    err2=midline2-63
    
    
#######################################


