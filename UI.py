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
import ticker

import gc
import time

# 定义控制引脚
rst = Pin('B9' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
dc  = Pin('B8' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
blk = Pin('C4' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
# 新建 LCD 驱动实例 这里的索引范围与 SPI 示例一致 当前仅支持 IPS200
drv = LCD_Drv(SPI_INDEX=1, BAUDRATE=60000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
# 新建 LCD 实例
lcd = LCD(drv)
# color 接口设置屏幕显示颜色 [前景色,背景色]
lcd.color(0xFFFF, 0x0000)
# mode 接口设置屏幕显示模式 [0:竖屏,1:横屏,2:竖屏180旋转,3:横屏180旋转]
lcd.mode(2)
# 清屏 参数是 RGB565 格式的颜色数据
lcd.clear(0x0000)


# 实例化 KEY_HANDLER 模块 参数是按键扫描周期
# 扫描周期根据实际 ticker 周期或者延时周期来确定
# 请务必确保扫描周期是正确的 否则按键触发可能会有问题
key = KEY_HANDLER(10)
key_set=1

def get_key():
    global key_set
    
    # 通过 capture 接口更新数据 但在这个例程中被 ticker 模块接管了
    # key.capture()
    # 通过 get 接口读取数据
    key_data = key.get()
    
    # 按键数据为三个状态 0-无动作 1-短按 2-长按
    if key_data[0]:
        print("key1 = {:>6d}.".format(key_data[0]))
        
        key_set=key_set-1
        lcd.clear(0x0000)
        
        key.clear(1)
    if key_data[1]:
        print("key2 = {:>6d}.".format(key_data[1]))
        
        key_set=key_set+1
        lcd.clear(0x0000)
        
        key.clear(2)
        
    if key_data[2]:
        print("key3 = {:>6d}.".format(key_data[2]))
        key.clear(3)
        
    if key_data[3]:
        print("key4 = {:>6d}.".format(key_data[3]))
        key.clear(4)


#显示赛道扫描到的线,并显示左中右
def lcdline():
    #显示扫描到的白道与黑道
    for i in range(len(camera.ccd2zh_data1)):
        if(camera.ccd2zh_data1[i]==0):
            lcd.line(i,60,i+1,60,color=0xFFFF,thick=1)
            lcd.line(i,61,i+1,61,color=0xFFFF,thick=1)
            lcd.line(i,62,i+1,62,color=0xFFFF,thick=1)
            lcd.line(i,63,i+1,63,color=0xFFFF,thick=1)
            lcd.line(i,64,i+1,64,color=0xFFFF,thick=1)
        elif(camera.ccd2zh_data1[i]==1):
            lcd.line(i,60,i+1,60,color=0X7FE0,thick=1)
            lcd.line(i,61,i+1,61,color=0X7FE0,thick=1)
            lcd.line(i,62,i+1,62,color=0X7FE0,thick=1)
            lcd.line(i,63,i+1,63,color=0X7FE0,thick=1)
            lcd.line(i,64,i+1,64,color=0X7FE0,thick=1)
        if(camera.ccd2zh_data2[i]==0):
            lcd.line(i,40,i+1,40,color=0xFFFF,thick=1)
            lcd.line(i,41,i+1,41,color=0xFFFF,thick=1)
            lcd.line(i,42,i+1,42,color=0xFFFF,thick=1)
            lcd.line(i,43,i+1,43,color=0xFFFF,thick=1)
            lcd.line(i,44,i+1,44,color=0xFFFF,thick=1)
        elif(camera.ccd2zh_data2[i]==1):
            lcd.line(i,40,i+1,40,color=0X7FE0,thick=1)
            lcd.line(i,41,i+1,41,color=0X7FE0,thick=1)
            lcd.line(i,42,i+1,42,color=0X7FE0,thick=1)
            lcd.line(i,43,i+1,43,color=0X7FE0,thick=1)
            lcd.line(i,44,i+1,44,color=0X7FE0,thick=1)
    
    #显示ccd1的左中右点
    for i in range(0,4,+1):
        lcd.line(findline.leftline,60+i,findline.leftline+1,60+i,color=0XF800,thick=2)
        lcd.line(findline.midline,60+i,findline.midline+1,60+i,color=0XF800,thick=2)
        lcd.line(findline.rightline,60+i,findline.rightline+1,60+i,color=0XF800,thick=2)
    #显示ccd2的左中右点
        lcd.line(findline.leftline2,40+i,findline.leftline2+1,40+i,color=0XF800,thick=2)
        lcd.line(findline.midline2,40+i,findline.midline2+1,40+i,color=0XF800,thick=2)
        lcd.line(findline.rightline2,40+i,findline.rightline2+1,40+i,color=0XF800,thick=2)
    #显示数据波形
    lcd.wave(0, 184, 128, 64, camera.ccd_data1)
    lcd.wave(0, 120, 128, 64, camera.ccd_data2)
                
        

def UI_process():
    global key_set
    
    if(key_set<=0):
        key_set=3
    elif(key_set>=4):
        key_set=1
    
    #赛道数据显示
    lcdline()
    
    
    if(key_set==1):
        test_check()#对陀螺仪与编码器进行检查
    elif(key_set==2):
        check_speedloop()#对速度环进行检查
    elif(key_set==3):
        loop_check()#对平衡环进行检查    
    
#     if(key_set==1):
#         check_state()#对元素状态进行检查
#     elif(key_set==2):
#         check_speedloop()#对速度环进行检查
#     elif(key_set==3):
#         turn_check()#对转向进行检查
#     elif(key_set==4):
#         over_check()#对完赛进行检查
        
################################调试#########################################################
def test_check():
    lcd.str16(140,12,"TEST",0XE000)
        
    #陀螺仪转向加速度
    lcd.str16(140,28,"z_angle",0xFFFF)
    lcd.str16(200,28,"{:f}.".format(gyroscope.angle_z),0xFFFF)
    lcd.str16(140,44,"z_gyro",0xFFFF)
    lcd.str16(200,44,"{:f}.".format(gyroscope.gyro_z),0xFFFF)
    lcd.str16(140,60,"z_sum",0xFFFF)
    lcd.str16(200,60,"{:f}.".format(gyroscope.gyro_zsum),0xFFFF)
    lcd.str16(140,76,"x_angle",0xFFFF)
    lcd.str16(200,76,"{:f}.".format(gyroscope.angle_x),0xFFFF)
    lcd.str16(140,92,"x_sum",0xFFFF)
    lcd.str16(200,92,"{:f}.".format(gyroscope.gyro_xsum),0xFFFF)
    lcd.str16(140,108,"y_angle",0xFFFF)
    lcd.str16(200,108,"{:f}.".format(gyroscope.angle_y),0xFFFF)
    lcd.str16(140,124,"y_sum",0xFFFF)
    lcd.str16(200,124,"{:f}.".format(gyroscope.gyro_ysum),0xFFFF)
    lcd.str16(140,140,"acc_x",0xFFFF)
    lcd.str16(200,140,"{:f}.".format(gyroscope.acc_x),0xFFFF)
    lcd.str16(140,156,"acc_y",0xFFFF)
    lcd.str16(200,156,"{:f}.".format(gyroscope.acc_y),0xFFFF)
    lcd.str16(140,172,"acc_z",0xFFFF)
    lcd.str16(200,172,"{:f}.".format(gyroscope.acc_z),0xFFFF)
    lcd.str16(140,188,"yaw",0xFFFF)
    lcd.str16(200,188,"{:f}.".format(gyroscope.yaw),0xFFFF)
    lcd.str16(140,204,"pitch",0xFFFF)
    lcd.str16(200,204,"{:f}.".format(gyroscope.pitch),0xFFFF)
    lcd.str16(140,220,"roll",0xFFFF)
    lcd.str16(200,220,"{:f}.".format(gyroscope.roll),0xFFFF)
    
def loop_check():
    lcd.str16(140,12,"Blance",0XE000)
        
    #陀螺仪转向加速度
    lcd.str16(140,28,"sped_o",0xFFFF)
    lcd.str16(200,28,"{:f}.".format(motor.speed_out),0xFFFF)
    lcd.str16(140,44,"agl_o",0xFFFF)
    lcd.str16(200,44,"{:f}.".format(motor.angle_out),0xFFFF)
    lcd.str16(140,60,"agl_so",0xFFFF)
    lcd.str16(200,60,"{:f}.".format(motor.angle_speed_out),0xFFFF)
    
    
#########################################################################################
    
    
def over_check():
    #完赛检查
    lcd.str16(140,12,"over",0XE000)
    
    #舵机角度
    lcd.str16(140,28,"time_s",0xFFFF)
    lcd.str16(200,28,"{:f}.".format(ticker.time_count/10),0xFFFF)
    
    #与中线偏差
    lcd.str16(140,44,"speed",0xFFFF)
    lcd.str16(200,44,"{:f}.".format(encode.speed_over),0xFFFF)

    #陀螺仪转向加速度
    lcd.str16(140,60,"length",0xFFFF)
    lcd.str16(200,60,"{:f}.".format(encode.lap_length),0xFFFF)

def turn_check():
    #转向调参
    lcd.str16(140,12,"angle",0XE000)
    
    #舵机角度
    lcd.str16(140,28,"angle",0xFFFF)
    lcd.str16(200,28,"{:f}.".format(servo.angle),0xFFFF)

    #与中线偏差
    lcd.str16(140,44,"err",0xFFFF)
    lcd.str16(200,44,"{:d}.".format(findline.err),0xFFFF)

    #陀螺仪转向加速度
    lcd.str16(140,60,"z_angle",0xFFFF)
    lcd.str16(200,60,"{:f}.".format(gyroscope.angle_z),0xFFFF)
    lcd.str16(140,76,"z_gyro",0xFFFF)
    lcd.str16(200,76,"{:f}.".format(gyroscope.gyro_z),0xFFFF)
    lcd.str16(140,92,"z_sum",0xFFFF)
    lcd.str16(200,92,"{:f}.".format(gyroscope.gyro_zsum),0xFFFF)
    lcd.str16(140,108,"x_gyro",0xFFFF)
    lcd.str16(200,108,"{:f}.".format(gyroscope.gyro_x),0xFFFF)
    lcd.str16(140,124,"x_sum",0xFFFF)
    lcd.str16(200,124,"{:f}.".format(gyroscope.gyro_xsum),0xFFFF)
    lcd.str16(140,140,"y_gyro",0xFFFF)
    lcd.str16(200,140,"{:f}.".format(gyroscope.gyro_y),0xFFFF)
    lcd.str16(140,156,"y_sum",0xFFFF)
    lcd.str16(200,156,"{:f}.".format(gyroscope.gyro_ysum),0xFFFF)

    
def check_speedloop():
    lcd.str16(140,12,"speed",0XE000)
    
    #速度环调参
    lcd.str16(140,28,"L_set",0xFFFF)
    lcd.str16(200,28,"{:f}.".format(motor.L_setSpeed),0xFFFF)
    lcd.str16(140,44,"R_set",0xFFFF)
    lcd.str16(200,44,"{:f}.".format(motor.R_setSpeed),0xFFFF)
    lcd.str16(140,60,"L_ctrl",0xFFFF)
    lcd.str16(200,60,"{:f}.".format(motor.L_SpeedControl),0xFFFF)
    lcd.str16(140,78,"R_ctrl",0xFFFF)
    lcd.str16(200,78,"{:f}.".format(motor.R_SpeedControl),0xFFFF)
    lcd.str16(140,94,"stop",0xFFFF)
    lcd.str16(200,94,"{:d}.".format(findline.stopflag),0xFFFF)
    
    #左右轮速度
    lcd.str16(140,110,"l_sped",0xFFFF)
    lcd.str16(200,110,"{:f}.".format(encode.L_CarSpeed),0xFFFF)
    lcd.str16(140,126,"r_sped",0xFFFF)
    lcd.str16(200,126,"{:f}.".format(encode.R_CarSpeed),0xFFFF)
    
    #编码器累加量
    lcd.str16(140,142,"l_ecdr",0xFFFF)
    lcd.str16(200,142,"{:d}.".format(encode.Left_count),0xFFFF)
    lcd.str16(140,158,"r_ecdr",0xFFFF)
    lcd.str16(200,158,"{:d}.".format(encode.Right_count),0xFFFF)
    lcd.str16(140,174,"l_sum",0xFFFF)
    lcd.str16(200,174,"{:d}.".format(encode.L_sum),0xFFFF)
    lcd.str16(140,190,"r_sum",0xFFFF)
    lcd.str16(200,190,"{:d}.".format(encode.R_sum),0xFFFF)
    lcd.str16(140,206,"l_duty",0xFFFF)
    lcd.str16(200,206,"{:d}.".format(motor.l_duty),0xFFFF)
    lcd.str16(140,222,"r_duty",0xFFFF)
    lcd.str16(200,222,"{:d}.".format(motor.r_duty),0xFFFF)
    
    #速度决策
    lcd.str16(140,238,"SpedSt",0xFFFF)
    lcd.str16(200,238,"{:f}.".format(motor.Speed_Stan),0xFFFF)
    
    
    
def check_state():
    lcd.str16(140,12,"state",0XE000)
    
    #陀螺仪惯性量
    lcd.str16(140,28,"gyro_z",0xFFFF)
    lcd.str16(200,28,"{:f}.".format(gyroscope.gyro_zsum),0xFFFF)

        
    #远端摄像头左中右
    lcd.str16(140,44,"lef2",0xFFFF)
    lcd.str16(140,60,"mid2",0xFFFF)
    lcd.str16(140,76,"rig2",0xFFFF)
    lcd.str16(200,44,"{:d}.".format(findline.leftline2),0xFFFF)
    lcd.str16(200,60,"{:d}.".format(findline.midline2),0xFFFF)
    lcd.str16(200,76,"{:d}.".format(findline.rightline2),0xFFFF)


    #近端摄像头左中右
    lcd.str16(140,92,"lef",0xFFFF)
    lcd.str16(140,108,"mid",0xFFFF)
    lcd.str16(140,124,"rig",0xFFFF)
    lcd.str16(200,92,"{:d}.".format(findline.leftline),0xFFFF)
    lcd.str16(200,108,"{:d}.".format(findline.midline),0xFFFF)
    lcd.str16(200,124,"{:d}.".format(findline.rightline),0xFFFF)
        
    #状态
    lcd.str16(140,140,"r_state",0xFFFF)
    lcd.str16(200,140,"{:d}.".format(findline.rightroundstate),0xFFFF)
    lcd.str16(140,156,"stait",0xFFFF)
    lcd.str16(200,156,"{:d}.".format(findline.straight),0xFFFF)
    lcd.str16(140,172,"cross",0xFFFF)
    lcd.str16(200,172,"{:d}.".format(findline.crossstate),0xFFFF)
    lcd.str16(140,188,"xiepo",0xFFFF)
    lcd.str16(200,188,"{:d}.".format(findline.xiepostate),0xFFFF)
    lcd.str16(140,204,"stop",0xFFFF)
    lcd.str16(200,204,"{:d}.".format(findline.stopflag),0xFFFF)
    lcd.str16(140,220,"l_state",0xFFFF)
    lcd.str16(200,220,"{:d}.".format(findline.leftroundstate),0xFFFF)
    
    
    lcd.str16(140,236,"length",0xFFFF)
    lcd.str16(200,236,"{:f}.".format(encode.lap_length),0xFFFF)
    lcd.str16(140,252,"r_rond",0xFFFF)
    lcd.str16(200,252,"{:f}.".format(findline.turn_angle_right),0xFFFF)
    
    lcd.str16(140,268,"l_rond",0xFFFF)
    lcd.str16(200,268,"{:f}.".format(findline.turn_angle_left),0xFFFF)
    
#     lcd.str16(140,284,"gyro_y",0xFFFF)
#     lcd.str16(200,284,"{:f}.".format(gyroscope.gyro_y),0xFFFF)
    
    lcd.str16(140,284,"l_long",0xFFFF)
    lcd.str16(200,284,"{:f}.".format(findline.left_length),0xFFFF)
    
    lcd.str16(140,300,"r_long",0xFFFF)
    lcd.str16(200,300,"{:f}.".format(findline.right_length),0xFFFF)
    
    