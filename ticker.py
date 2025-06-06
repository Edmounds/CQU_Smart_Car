from machine import *

from smartcar import *
from seekfree import *
from display import *

import encode
import camera
import UI
import servo
import motor
import findline
import gyroscope

import gc
import time

time_count=1#对完赛时间进行计数

####################################2ms ticker###########################################
def time_2ms_pit_init():
    # 实例化 PIT  模块 参数为编号 [0-3] 最多四个
    pit2 = ticker(2)
    # 关联采集接口 最少一个 最多八个 (imu, ccd, key...)
    # 可关联 smartcar 的 ADC_Group_x 与 encoder_x
    # 可关联 seekfree 的  IMU660RA, IMU963RA, KEY_HANDLER 和 TSL1401
    pit2.capture_list(gyroscope.imu)
    # 关联 Python 回调函数
    pit2.callback(time_2ms_pit_handler)
    # 启动 ticker 实例 参数是触发周期 单位是毫秒
    pit2.start(2)
    
def time_2ms_pit_handler(time):
    
    gyroscope.gyroscope_process()
    servo.servoprocess()
#######################################
    

####################################10ms ticker###########################################
def time_10ms_pit_init():
    # 实例化 PIT  模块 参数为编号 [0-3] 最多四个
    pit0 = ticker(0)
    # 关联采集接口 最少一个 最多八个 (imu, ccd, key...)
    # 可关联 smartcar 的 ADC_Group_x 与 encoder_x
    # 可关联 seekfree 的  IMU660RA, IMU963RA, KEY_HANDLER 和 TSL1401
    pit0.capture_list(encode.encoder_l, encode.encoder_r, camera.ccd,  UI.key)
    # 关联 Python 回调函数
    pit0.callback(time_10ms_pit_handler)
    # 启动 ticker 实例 参数是触发周期 单位是毫秒
    pit0.start(10)
    
def time_10ms_pit_handler(time):
    global time_count
    
    UI.get_key()
    
    servo.servoprocess()
    encode.getspeed()
    #motor.Speed_out_Loop()
    if(time_count>2):
        motor.balance_loop_process()
    
    camera.cameraprocess()
    findline.erzhihua_findline()
    findline.caculate_err()
    
#######################################
    

####################################100ms ticker##########################################
    
def time_100ms_pit_init():
    # 实例化 PIT  模块 参数为编号 [0-3] 最多四个
    pit1 = ticker(1)
    # 关联采集接口 最少一个 最多八个 (imu, ccd, key...)
    # 可关联 smartcar 的 ADC_Group_x 与 encoder_x
    # 可关联 seekfree 的  IMU660RA, IMU963RA, KEY_HANDLER 和 TSL1401
#     pit1.capture_list()
    # 关联 Python 回调函数
    pit1.callback(time_100ms_pit_handler)
    # 启动 ticker 实例 参数是触发周期 单位是毫秒
    pit1.start(100)

def time_100ms_pit_handler(time):
    global time_count
    
    
    UI.UI_process()
    time_count=time_count+1
        
#######################################




    

