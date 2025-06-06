from machine import *

from smartcar import *
from seekfree import *
from display import *

import encode
import ticker
import UI
import camera
import motor
import findline

import gc
import time
import math

# 调用 IMU660RA 模块获取 IMU660RA 实例
# 参数是采集周期 调用多少次 capture 更新一次数据
# 可以不填 默认参数为 1 调整这个参数相当于调整采集分频
imu = IMU660RA()
gyro_zsum=0
gyro_xsum=0
gyro_ysum=0
gyro_y=0
gyro_x=0
gyro_z=0
angle_z=0
last_angle_z=[0,0,0,0,0]



##################################四元数解算角度##################################
# 定义常量
Kp = 10.0  # 比例增益
Ki = 0.008  # 积分增益
halfT = 0.001  # 采样周期的一半，假设采样频率为500Hz
# 静态变量
q0, q1, q2, q3 = 1, 0, 0, 0  # 四元数元素
exInt, eyInt, ezInt = 0, 0, 0  # 积分误差
yaw, pitch, roll = 0, 0, 0  # 欧拉角
def IMU_Update(gx, gy, gz, ax, ay, az):
    global q0, q1, q2, q3, exInt, eyInt, ezInt, yaw, pitch, roll
    
    if ax * ay * az == 0:
        return
    
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    ax /= norm
    ay /= norm
    az /= norm
    
    vx = 2 * (q1 * q3 - q0 * q2)
    vy = 2 * (q0 * q1 + q2 * q3)
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3
    
    ex = (ay * vz - az * vy)
    ey = (az * vx - ax * vz)
    ez = (ax * vy - ay * vx)
    
    exInt += ex * Ki
    eyInt += ey * Ki
    ezInt += ez * Ki
    
    gx += Kp * ex + exInt
    gy += Kp * ey + eyInt
    gz += Kp * ez + ezInt
    
    temp0 = q0
    temp1 = q1
    temp2 = q2
    temp3 = q3
    
    q0 += (-temp1 * gx - temp2 * gy - temp3 * gz) * halfT
    q1 += (temp0 * gx + temp2 * gz - temp3 * gy) * halfT
    q2 += (temp0 * gy - temp1 * gz + temp3 * gx) * halfT
    q3 += (temp0 * gz + temp1 * gy - temp2 * gx) * halfT
    
    norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
    q0 /= norm
    q1 /= norm
    q2 /= norm
    q3 /= norm
    
    yaw = math.atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 57.3
    pitch = math.asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3
    roll = math.atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3

    # 示例调用
    # IMU_Update(0.1, 0.2, 0.3, 0.9, 0.8, 0.7)
    #print("Yaw:", yaw, "Pitch:", pitch, "Roll:", roll)
##################################
    
    
##################################一阶惯性低通滤波器##################################
def LPF_1_db(receive,out,hz=500,time=2):
    out = out + ( 1 / ( 1 + 1 / ( hz * 6.28 * time ) ) ) * ( receive - out )
    return out
##################################

##################################陀螺仪解算程序##################################
def gyroscope_process():
    global imu_data
    global gyro_zsum
    global gyro_ysum
    global gyro_xsum
    global gyro_z#调试用
    global gyro_x#调试用
    global gyro_y
    global angle_z
    global angle_y
    global angle_x
    
    global acc_x
    global acc_y
    global acc_z
    
    
    global last_angle_z
    
    imu_data = imu.get()
    
    #加速度
    acc_x=imu_data[0]
    acc_y=imu_data[1]
    acc_z=imu_data[2]
    #加速度，单位米每秒的平方
    acc_x=acc_x/415#自己测量
    acc_y=acc_y/415
    acc_z=acc_z/415
    
    gyro_x=LPF_1_db(receive=imu_data[3],out=gyro_x)
    gyro_y=LPF_1_db(receive=imu_data[4],out=gyro_y)
    gyro_z=LPF_1_db(receive=imu_data[5]+2.2,out=gyro_z)#这里+2.2是因为仪器本身有误差，手动对其调零
    
    #角速度，单位度
    angle_z=gyro_z*0.0001455
    angle_x=gyro_x*0.0001455
    angle_y=gyro_y*0.0001455
    
    
    #转向角，单位度
    gyro_zsum=angle_z+gyro_zsum
    gyro_ysum=angle_y+gyro_ysum
    gyro_xsum=angle_x+gyro_xsum
    
    IMU_Update(angle_z, angle_z, angle_z, acc_x, acc_y, acc_z)
##################################