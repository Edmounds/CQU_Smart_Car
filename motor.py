from machine import *


from smartcar import *
from seekfree import *

import encode
import ticker
import UI
import camera
import motor
import findline
import servo
import gyroscope

import time
import gc


#————————————————————驱动板为前后转信号线控制—————————————————————————————————
motor_l_forward = PWM('C26', freq=15000, duty_ns=0)
motor_l_backward = PWM('C27', freq=15000, duty_ns=0)

motor_r_forward = PWM('C24', freq=15000, duty_ns=0)
motor_r_backward = PWM('C25', freq=15000, duty_ns=0)
#————————————————————————————————————————————————————————————


# #————————————————————驱动板为高低电平控制—————————————————————————————————
# motor_l_ward = PWM('C25', freq=15000, duty_ns=0)
# motor_l_dir=Pin('C27', Pin.OUT)
# motor_l_dir.off()
# 
# motor_r_ward = PWM('C24', freq=15000, duty_ns=0)
# motor_r_dir=Pin('C26', Pin.OUT)
# motor_r_dir.off()
# #————————————————————————————————————————————————————————————

motor_max = 50000
motor_min = -50000 #电机范围只能在-50000~50000


#————————————————————速度输出pid—————————————————————————————————
#11.1v
L_SpeedLoop_KP = 0.4#0.6
L_SpeedLoop_KI = 0#0.07
L_SpeedLoop_KD = 0.7#0.7

R_SpeedLoop_KP = 0.4 #0.6
R_SpeedLoop_KI = 0#0.07
R_SpeedLoop_KD = 0.7#0.7

#7.4v
# L_SpeedLoop_KP = 0.6
# L_SpeedLoop_KI = 0.07
# L_SpeedLoop_KD = 0.7
# 
# R_SpeedLoop_KP = 0.6
# R_SpeedLoop_KI = 0.07
# R_SpeedLoop_KD = 0.7
#————————————————————————————————————————————————————————————

#—————————————————————速度———————————————————————————————————
Speed_Set=1.5#设定小车额定速度为__m/s
#————————————————————————————————————————————————————————————

########################################输出速度环PID##########################################
L_SpeedLoop_ErrorFifo=[0,0,0,0]#存储量（用于积分和微分）
L_SpeedLoop_ErrorDtFifo=[0,0,0,0]#微分的存储量
L_SpeedLoop_Error=0
L_SpeedLoop_Integ=0#积分量
L_SpeedControl=0
L_SpeedLoop_OutPut=0

R_SpeedLoop_ErrorFifo=[0,0,0,0]#存储量（用于积分和微分）
R_SpeedLoop_ErrorDtFifo=[0,0,0,0]#微分的存储量
R_SpeedLoop_Error=0
R_SpeedLoop_Integ=0#积分量
R_SpeedControl=0
R_SpeedLoop_OutPut=0

L_setSpeed=0
L_setSpeed=0

def Speed_out_Loop():
    global L_SpeedLoop_ErrorFifo
    global L_SpeedLoop_ErrorDtFifo
    global Speed_Stan
    global Speed_Set
    global L_SpeedLoop_Error
    global L_SpeedLoop_Integ
    global L_SpeedLoop_KP
    global L_SpeedLoop_KI
    global L_SpeedLoop_KD
    global L_SpeedLoop_OutPut
    global L_SpeedControl
    
    global R_SpeedLoop_ErrorFifo
    global R_SpeedLoop_ErrorDtFifo
    global R_Speed_Stan
    global R_SpeedLoop_Error
    global R_SpeedLoop_Integ
    global R_SpeedLoop_KP
    global R_SpeedLoop_KI
    global R_SpeedLoop_KD
    global R_SpeedLoop_OutPut
    global R_SpeedControl
    
    
    global L_setSpeed
    global R_setSpeed
    global angle_speed_out
#———————————————————————————————————————————停车代码—————————————————————————————————————————————————————
    if(gyroscope.pitch>=-45):
        findline.stopflag=1
        
    if(findline.stopflag==0):
        Speed_Stan=2#测试
        
        Speed_Stan=Speed_Set#速度是设定速度和速度决策出来的速度
    else:
        Speed_Stan=0
#————————————————————————————————————————
        
    
#———————————————————————————————————————差速计算—————————————————————————————————————————————————————————————
    
    rate_angle=0.02 #转向pid系数调整0.02
    delta_pout=servo.angle#正左负右
    
    L_setSpeed=Speed_Stan*(1-rate_angle*delta_pout)#-rate_speed*abs(findline.err)-rate_speed2*abs(findline.err2)
    R_setSpeed=Speed_Stan*(1+rate_angle*delta_pout)#-rate_speed*abs(findline.err)-rate_speed2*abs(findline.err2)
#————————————————————————————————————————————————————————————————————————————————————————————————

    
    
    
#——————————————————————————————————左轮速度环————————————————————————————————————————————————————————————————————
    
    
    ki_max = 0.5 #积分量限幅值
    
    SpeedLoop_ErrorMaxLimit = 2
    SpeedLoop_ErrorMinLimit = -2
    
    SpeedLoop_OutPutMaxLimit=50000
    SpeedLoop_OutPutMinLimit=-50000

    L_SpeedLoop_Error = L_setSpeed - encode.L_CarSpeed
    
    L_SpeedLoop_ErrorFifo[2] = L_SpeedLoop_ErrorFifo[1]
    L_SpeedLoop_ErrorFifo[1] = L_SpeedLoop_ErrorFifo[0]
    L_SpeedLoop_ErrorFifo[0] = L_SpeedLoop_Error

    L_SpeedLoop_ErrorDtFifo[2] = L_SpeedLoop_ErrorDtFifo[1];
    L_SpeedLoop_ErrorDtFifo[1] = L_SpeedLoop_ErrorDtFifo[0];
    L_SpeedLoop_ErrorDtFifo[0] = L_SpeedLoop_ErrorFifo[0] - L_SpeedLoop_ErrorFifo[2];

    if(L_SpeedLoop_Error>=SpeedLoop_ErrorMaxLimit):
        L_SpeedLoop_Error = SpeedLoop_ErrorMaxLimit
    if(L_SpeedLoop_Error<=SpeedLoop_ErrorMinLimit):
        L_SpeedLoop_Error = SpeedLoop_ErrorMinLimit
        
    if(abs(L_SpeedLoop_Error<=1)):
        L_SpeedLoop_Integ = L_SpeedLoop_Integ + L_SpeedLoop_Error
        
    if(L_SpeedLoop_Integ > ki_max):
        L_SpeedLoop_Integ = ki_max   #对积分量进行限幅
    if(L_SpeedLoop_Integ <-ki_max):
        L_SpeedLoop_Integ = -ki_max
    
    L_SpeedLoop_OutPut = L_SpeedLoop_KP * L_SpeedLoop_Error + L_SpeedLoop_KI * L_SpeedLoop_Integ + L_SpeedLoop_KD * (L_SpeedLoop_ErrorDtFifo[0] * 0.6 + L_SpeedLoop_ErrorDtFifo[1] * 0.4)

    L_SpeedLoop_OutPut =  L_SpeedLoop_OutPut * 25000
    
    if(L_SpeedLoop_OutPut>=SpeedLoop_OutPutMaxLimit):
        L_SpeedLoop_OutPut = SpeedLoop_OutPutMaxLimit
    if(L_SpeedLoop_OutPut<=SpeedLoop_OutPutMinLimit):
        L_SpeedLoop_OutPut = SpeedLoop_OutPutMinLimit

    L_SpeedControl = int( 0.2 * L_SpeedLoop_OutPut + 0.8 * L_SpeedControl)#闭环输出
#     L_SpeedControl =10000 #开环输出
    

    if(L_SpeedControl>=SpeedLoop_OutPutMaxLimit):
        L_SpeedControl = SpeedLoop_OutPutMaxLimit
    if(L_SpeedControl<=SpeedLoop_OutPutMinLimit):
        L_SpeedControl = SpeedLoop_OutPutMinLimit

#————————————————————————————————————————————————————————————————————————————————————————————————

#——————————————————————————————————右轮速度环————————————————————————————————————————————————————————————————————
    

    R_SpeedLoop_Error = R_setSpeed - encode.R_CarSpeed 

    R_SpeedLoop_ErrorFifo[2] = R_SpeedLoop_ErrorFifo[1]
    R_SpeedLoop_ErrorFifo[1] = R_SpeedLoop_ErrorFifo[0]
    R_SpeedLoop_ErrorFifo[0] = R_SpeedLoop_Error

    R_SpeedLoop_ErrorDtFifo[2] = R_SpeedLoop_ErrorDtFifo[1];
    R_SpeedLoop_ErrorDtFifo[1] = R_SpeedLoop_ErrorDtFifo[0];
    R_SpeedLoop_ErrorDtFifo[0] = R_SpeedLoop_ErrorFifo[0] - R_SpeedLoop_ErrorFifo[2];

    if(R_SpeedLoop_Error>=SpeedLoop_ErrorMaxLimit):
        R_SpeedLoop_Error = SpeedLoop_ErrorMaxLimit
    if(R_SpeedLoop_Error<=SpeedLoop_ErrorMinLimit):
        R_SpeedLoop_Error = SpeedLoop_ErrorMinLimit
        
    if(abs(R_SpeedLoop_Error<=1)):
        R_SpeedLoop_Integ = R_SpeedLoop_Integ + R_SpeedLoop_Error
        
    if(R_SpeedLoop_Integ > ki_max):
        R_SpeedLoop_Integ = ki_max   #对积分量进行限幅
    if(R_SpeedLoop_Integ <-ki_max):
        R_SpeedLoop_Integ = -ki_max
    
    R_SpeedLoop_OutPut = R_SpeedLoop_KP * R_SpeedLoop_Error + R_SpeedLoop_KI * R_SpeedLoop_Integ + R_SpeedLoop_KD * (R_SpeedLoop_ErrorDtFifo[0] * 0.6 + R_SpeedLoop_ErrorDtFifo[1] * 0.4)

    R_SpeedLoop_OutPut =  R_SpeedLoop_OutPut * 25000
    
    if(R_SpeedLoop_OutPut>=SpeedLoop_OutPutMaxLimit):
        R_SpeedLoop_OutPut = SpeedLoop_OutPutMaxLimit
    if(R_SpeedLoop_OutPut<=SpeedLoop_OutPutMinLimit):
        R_SpeedLoop_OutPut = SpeedLoop_OutPutMinLimit

    R_SpeedControl = int( 0.2 * R_SpeedLoop_OutPut + 0.8 * R_SpeedControl)#闭环输出
#     R_SpeedControl =10000 #开环输出

    if(R_SpeedControl>=SpeedLoop_OutPutMaxLimit):
        R_SpeedControl = SpeedLoop_OutPutMaxLimit
    if(R_SpeedControl<=SpeedLoop_OutPutMinLimit):
        R_SpeedControl = SpeedLoop_OutPutMinLimit

#————————————————————————————————————————————————————————————————————————————————————————————————



#######################################

#######################################输出限幅############################################
def PWM_limit(duty,duty_max,duty_min):
    if(duty>duty_max):
        duty = duty_max
    elif (duty<duty_min):
        duty = duty_min
    return int(duty)
########################################

########################################平衡环计算########################################
def balance_loop_process():
    
    Speed_out_Loop()
    Motor_0ut(duty_left=L_SpeedControl, duty_right=R_SpeedControl)
########################################

l_duty=0
r_duty=0

########################################电机输出##########################################

def Motor_0ut(duty_left, duty_right):
    global motor_max
    global motor_min
    
    global l_duty
    global r_duty
    
    
    l_duty=PWM_limit(duty_left,motor_max,motor_min)
    r_duty=PWM_limit(duty_right,motor_max,motor_min)
    
    #———————————————对于11.4V的电压需要进行一个2/3的限幅（电机额定电压为7.6V）————————————————————————————————————
    l_duty=int(l_duty*2/3)
    r_duty=int(r_duty*2/3)
    #—————————————————————————————————————————————————————————————————————————————————————————————————————————
    
    #————————————————————驱动板为前后转信号线控制—————————————————————————————————
    if(l_duty>0):#左轮输出
        motor_l_forward.duty_ns(l_duty)
        motor_l_backward.duty_ns(0)
    elif(l_duty<0):
        motor_l_forward.duty_ns(0)
        motor_l_backward.duty_ns(abs(l_duty))
    elif(l_duty==0):
        motor_l_forward.duty_ns(0)
        motor_l_backward.duty_ns(0)
        
    if(r_duty>0):#右轮输出
        motor_r_forward.duty_ns(r_duty)
        motor_r_backward.duty_ns(0)
    elif(r_duty<0):
        motor_r_forward.duty_ns(0)
        motor_r_backward.duty_ns(abs(r_duty))        
    elif(r_duty==0):
        motor_r_forward.duty_ns(0)
        motor_r_backward.duty_ns(0)
    #—————————————————————————————————————————————————————————————————————————————

    #————————————————————驱动板为高低电平控制—————————————————————————————————
#     #左轮输出
#     if(l_duty>0):#向前
#         motor_l_ward.duty_ns(l_duty)
#         motor_l_dir.off()
#     elif(l_duty<0):#向后
#         motor_l_ward.duty_ns(abs(l_duty))
#         motor_l_dir.on()
#     elif(l_duty==0):#停止
#         motor_l_ward.duty_ns(0)
#         motor_l_dir.off()
#     
#     
#     #右轮输出
#     if(r_duty>0):#向前
#         motor_r_ward.duty_ns(r_duty)
#         motor_r_dir.off()
#     elif(r_duty<0):#向后
#         motor_r_ward.duty_ns(abs(r_duty))
#         motor_r_dir.on()
#     elif(r_duty==0):#停止
#         motor_r_ward.duty_ns(0)
#         motor_r_dir.off()
    
    
#######################################
