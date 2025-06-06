from machine import *
from display import *
from smartcar import *
from seekfree import *
import gc
import time
import math
import ustruct

err_1 = 0
err_sum_1 = 0
err_x_1 = 0
err_last_1 = 0
err_2 = 0
err_sum_2 = 0
err_x_2 = 0
err_last_2 = 0
err_3 = 0
err_sum_3 = 0
err_x_3 = 0
err_last_3 = 0
errt = 0
errt_sum = 0
errt_x = 0
errt_last = 0
min_value = 0
max_value = 0
loud = 100
ticker_count0 = 0
ticker_flag2 = 0
Filter_data = [0, 0, 0]
PI = 3.14159265358
last_yaw = 0
end_switch = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
end_state = end_switch.value()

# 定义片选引脚
fenmingqi = Pin('D24', Pin.OUT, pull=Pin.PULL_UP_47K, value=0)

cs = Pin('B29', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
# 拉高拉低一次 CS 片选确保屏幕通信时序正常
cs.high()
cs.low()
key = KEY_HANDLER(10)
# 定义控制引脚
rst = Pin('B31', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
dc = Pin('B5', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
blk = Pin('C21', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)

# 新建 LCD 驱动实例
drv = LCD_Drv(SPI_INDEX=1, BAUDRATE=60000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
# 新建 LCD 实例
lcd = LCD(drv)
# color 接口设置屏幕显示颜色 [前景色,背景色]
lcd.color(0x0000, 0xFFFF)
# mode 接口设置屏幕显示模式 [0:竖屏,1:横屏,2:竖屏180旋转,3:横屏180旋转]
lcd.mode(0)
# 清屏
lcd.clear(0xFFFF)
# 电机初始化
motor_l = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_PWM_C29, 13000, duty=0, invert=True)
motor_r = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D6_DIR_D7, 13000, duty=0, invert=True)
motor_dir = 1
motor_duty = 0
motor_duty_max = 1000


turn = 0
led = Pin('C4', Pin.OUT, pull=Pin.PULL_UP_47K, value=True)
# 编码器初始化
encoder_l = encoder("D0", "D1", True)
encoder_r = encoder("D2", "D3")


# 调用 TSL1401 模块获取 CCD 实例
# 参数是采集周期 调用多少次 capture 更新一次数据
# 默认参数为 1 调整这个参数相当于调整曝光时间倍数
ccd = TSL1401(10)
# 陀螺仪初始化
imu = IMU963RA()
imu_data = imu.get()

ticker_flag = False
ticker_count2 = 0
ticker_count = 0
ticker_count3 = 0
runtime_count = 0
i = 0
##############################################
rline2_last = 0
lline2_last = 0
banma_flag = 0
stop = 1
ting = 0
ysbzw = 1
lline_track = 0
rline_track = 0
ccd_sum = 10
track_stop = 0
huang_tag = 0
huang_l_flag = 0
huang_r_flag = 0
huang_l_zt = 0
huang_r_zt = 0
bizhan_l = 0
bizhan_r = 0
lline_last = 0
rline_last = 0
huang_l_zt2_min = 0
huang_r_zt2_max = 0
zebra = 0
road2 = 0


##############################################UART
uart2 = UART(2)
uart2.init(115200)

distance = 0
id_number = 0  # 初始化变量m为整数

# 串级参数
# ////////////角速度//////////////////////
angle_kp = -1270.0  # 1870
angle_ki = -20.00  # 20.26
angle_kd = -180.0  # 400
# ////////////角度//////////////////////
roll_angle_Kp = 0.0682  # 0.1002
roll_angle_Ki = 0.000001  # 0.00081
roll_angle_Kd = 0.1011  # 0.2011
# ////////////速度//////////////////////
speed_Kp = 0.065  # 0.07
speed_Ki = 0.00012  # 0.0012
speed_Kd = 0.0
angle_1 = 0
speed_1 = 0
counts = 0
motor1 = 0
motor2 = 0
med_roll_angle = 32
med_speed1 = 40

# ////////////////转向//////////////////////
a = 0.0032  # 0.026
Kpc = 1.781  # 7.721
turn_ki = 0.008  # 0.014
turn_kd = 15.64  # 0.62
turn_kd2 = -110.12

#################################################编码器卡尔曼滤波
KAL_P = 0.02  # 估算协方差
KAL_G = 0.0  # 卡尔曼增益
KAL_Q = 0.70  # 过程噪声协方差,Q增大，动态响应变快，收敛稳定性变坏
KAL_R = 200  # 测量噪声协方差,R增大，动态响应变慢，收敛稳定性变好
KAL_Output = 0.0  # 卡尔曼滤波器输出

KAL_P2 = 0.02  # 估算协方差
KAL_G2 = 0.0  # 卡尔曼增益
KAL_Q2 = 0.70  # 过程噪声协方差,Q增大，动态响应变快，收敛稳定性变坏
KAL_R2 = 200  # 测量噪声协方差,R增大，动态响应变慢，收敛稳定性变好
KAL_Output2 = 0.0  # 卡尔曼滤波器输出


######################################################
def limit(value, min_value, max_value):
    if value < min_value:
        value = min_value
    elif value > max_value:
        value = max_value
    else:
        value = value
    return value

# 位置式PID控制
def calculate_pid(err, err_sum, err_last, med, value, kp, ki, kd):
    """
    辅助函数，用于计算PID控制的基本部分，提取公共逻辑
    """
    err = med - value
    err_sum += err
    err_x = err - err_last
    pwm = kp * err + ki * err_sum + kd * err_x
    err_last = err
    return pwm, err, err_sum, err_last


def pid_position_1(med, value, kp, ki, kd):
    global err_1, err_sum_1, err_x_1, err_last_1
    pwm_1, err_1, err_sum_1, err_last_1 = calculate_pid(err_1, err_sum_1, err_last_1, med, value, kp, ki, kd)
    err_last_1 = err_1
    return pwm_1


def pid_position_2(med, value, kp, ki, kd):
    global err_2, err_sum_2, err_x_2, err_last_2
    pwm_2, err_2, err_sum_2, err_last_2 = calculate_pid(err_2, err_sum_2, err_last_2, med, value, kp, ki, kd)
    err_last_2 = err_2
    return pwm_2


def pid_position_3(med, value, kp, ki, kd):
    global err_3, err_sum_3, err_x_3, err_last_3
    pwm_3, err_3, err_sum_3, err_last_3 = calculate_pid(err_3, err_sum_3, err_last_3, med, value, kp, ki, kd)
    err_last_3 = err_3
    return pwm_3


def pid_turn(med, value, kp, ki, kd):
    global errt, errt_sum, errt_x, errt_last
    pwmt = calculate_pid(errt, errt_sum, errt_last, med, value, kp, ki, kd)
    errt_last = errt
    return pwmt


class bianmaqi:
    def __init__(self, KAL_templ_pluse, KAL_tempr_pluse):
        self.KAL_templ_pluse = KAL_templ_pluse
        self.KAL_tempr_pluse = KAL_tempr_pluse


Encoders = bianmaqi(0, 0)


class Imu_element:
    def __init__(self, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, Pitch, Roll, Yaw, X, Y, Z, Total_Yaw):
        self.acc_x = acc_x
        self.acc_y = acc_y
        self.acc_z = acc_z
        self.gyro_x = gyro_x
        self.gyro_y = gyro_y
        self.gyro_z = gyro_z
        self.Pitch = Pitch
        self.Roll = Roll
        self.Yaw = Yaw
        self.X = X
        self.Y = Y
        self.Z = Z
        self.Total_Yaw = Total_Yaw

Imu = Imu_element(0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00)

#############################################################
class param():
    def __init__(self, param_Kp, param_Ki):
        self.param_Kp = param_Kp
        self.param_Ki = param_Ki

Param = param(15.5, 0.006)


class QInfo:
    def __init__(self):
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0


Q_info = QInfo()
delta_T = 0.001  # 假设采样周期为5ms
I_ex, I_ey, I_ez = 0.0, 0.0, 0.0  # 积分误差


def invSqrt(x):
    return 1.0  # / (math.sqrt(x))


#############################################################
max_gyro_x = 0


############################陀螺仪############################

def Limit(value):
    if value > 1:
        value = 1
    elif value < -1:
        value = -1
    return value

# 姿态解算函数
def Imu963():
    alpha = 0.3
    global imu_data, max_gyro_x
    if abs(imu_data[3]) < 30 or abs(imu_data[3]) > 30000:
        imu_data[3] = 0
    if abs(imu_data[4]) < 30 or abs(imu_data[4]) > 30000:
        imu_data[4] = 0
    if abs(imu_data[5]) < 30 or abs(imu_data[5]) > 30000:
        imu_data[5] = 0

    Imu.X = int(imu_data[3] / 16.4)
    Imu.Y = int(imu_data[4] / 16.4)  # 俯仰角
    Imu.Z = int(imu_data[5] / 16.4)
    Imu.gyro_x = round((float(imu_data[3]) - Filter_data[0]), 3) * PI / 180 / 16.4
    Imu.gyro_y = round((float(imu_data[4]) - Filter_data[1]), 3) * PI / 180 / 16.4
    Imu.gyro_z = round((float(imu_data[5]) - Filter_data[2]), 3) * PI / 180 / 14.4
    Imu.acc_x = round(((float(imu_data[0]) * alpha) / 4096 + Imu.acc_x * (1 - alpha)), 3)
    Imu.acc_y = round(((float(imu_data[1]) * alpha) / 4096 + Imu.acc_y * (1 - alpha)), 3)
    Imu.acc_z = round(((float(imu_data[2]) * alpha) / 4096 + Imu.acc_z * (1 - alpha)), 3)
    # 四元素调用
    IMU_AHRSupdate(Imu.gyro_x, Imu.gyro_y, Imu.gyro_z, Imu.acc_x, Imu.acc_y, Imu.acc_z)
    if abs(max_gyro_x) < abs(Imu.Pitch):
        max_gyro_x = Imu.Pitch

# 陀螺仪初始化
def Imu963ra_Init():
    global Filter_data
    global imu_data
    Filter_data[0] = 0
    Filter_data[1] = 0
    Filter_data[2] = 0

    for i in range(0, 1000):
        imu_data = imu.get()
        Filter_data[0] += imu_data[3]
        Filter_data[1] += imu_data[4]
        Filter_data[2] += imu_data[5]
        time.sleep_ms(1)
    Filter_data[0] = float(Filter_data[0] / 1000)
    Filter_data[1] = float(Filter_data[1] / 1000)
    Filter_data[2] = float(Filter_data[2] / 1000)

# 四元数
def IMU_AHRSupdate(gx, gy, gz, ax, ay, az):
    global I_ex, I_ey, I_ez, last_yaw
    halfT = 0.5 * delta_T
    value1 = 0
    # 当前的机体坐标系上的重力单位向量
    vx, vy, vz = 0.0, 0.0, 0.0
    ex, ey, ez = 0.0, 0.0, 0.0
    q0, q1, q2, q3 = 0.0, 0.0, 0.0, 0.0

    q0q0 = Q_info.q0 * Q_info.q0
    q0q1 = Q_info.q0 * Q_info.q1
    q0q2 = Q_info.q0 * Q_info.q2
    q1q1 = Q_info.q1 * Q_info.q1
    q1q3 = Q_info.q1 * Q_info.q3
    q2q2 = Q_info.q2 * Q_info.q2
    q2q3 = Q_info.q2 * Q_info.q3
    q3q3 = Q_info.q3 * Q_info.q3

    # 对加速度数据进行归一化
    norm = invSqrt(ax * ax + ay * ay + az * az)
    ax *= norm
    ay *= norm
    az *= norm

    # 计算当前重力单位向量
    vx = 2 * (q1q3 - q0q2)
    vy = 2 * (q0q1 + q2q3)
    vz = q0q0 - q1q1 - q2q2 + q3q3

    # 计算误差
    ex = ay * vz - az * vy
    ey = az * vx - ax * vz
    ez = ax * vy - ay * vx

    # 用误差进行PI修正
    I_ex += delta_T * ex  # 积分误差
    I_ey += delta_T * ey
    I_ez += delta_T * ez

    gx += Param.param_Kp * ex + Param.param_Ki * I_ex
    gy += Param.param_Kp * ey + Param.param_Ki * I_ey
    gz += Param.param_Kp * ez + Param.param_Ki * I_ez

    # 四元数微分方程
    q0 = Q_info.q0
    q1 = Q_info.q1
    q2 = Q_info.q2
    q3 = Q_info.q3

    Q_info.q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT
    Q_info.q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT
    Q_info.q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT
    Q_info.q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT

    # 归一化四元数
    norm = invSqrt(Q_info.q0 ** 2 + Q_info.q1 ** 2 + Q_info.q2 ** 2 + Q_info.q3 ** 2)
    Q_info.q0 *= norm
    Q_info.q1 *= norm
    Q_info.q2 *= norm
    Q_info.q3 *= norm

    # 计算欧拉角
    value1 = Limit(-2 * Q_info.q1 * Q_info.q3 + 2 * Q_info.q0 * Q_info.q2)
    Imu.Roll = round(math.asin(value1) * 180 / math.pi, 3)  # pitch
    Imu.Pitch = round(math.atan2(2 * Q_info.q2 * Q_info.q3 + 2 * Q_info.q0 * Q_info.q1,
                                 -2 * Q_info.q1 ** 2 - 2 * Q_info.q2 ** 2 + 1) * 180 / math.pi, 3)  # roll
    Imu.Yaw = round(math.atan2(2 * Q_info.q1 * Q_info.q2 + 2 * Q_info.q0 * Q_info.q3,
                               -2 * Q_info.q2 ** 2 - 2 * Q_info.q3 ** 2 + 1) * 180 / math.pi, 3)  # yaw
    # 计算偏航角的误差
    error_yaw = Imu.Yaw - last_yaw
    if error_yaw < -360:
        error_yaw += 360
    if error_yaw > 360:
        error_yaw -= 360
    Imu.Total_Yaw += error_yaw
    last_yaw = Imu.Yaw

    # 保证total_yaw在0到360度之间
    if Imu.Total_Yaw > 360:
        Imu.Total_Yaw -= 360
    if Imu.Total_Yaw < 0:
        Imu.Total_Yaw += 360



def KalmanFilter(input):
    global KAL_P
    global KAL_G
    global KAL_Output

    KAL_P = KAL_P + KAL_Q  # 估算协方差方程：当前 估算协方差 = 上次更新 协方差 + 过程噪声协方差
    KAL_G = KAL_P / (KAL_P + KAL_R)  # //卡尔曼增益方程：当前 卡尔曼增益 = 当前 估算协方差 / （当前 估算协方差 + 测量噪声协方差）
    # 更新最优值方程：当前 最优值 = 当前 估算值 + 卡尔曼增益 * （当前 测量值 - 当前 估算值）
    KAL_Output = KAL_Output + KAL_G * (input - KAL_Output)  # 当前 估算值 = 上次 最优值
    KAL_P = (1 - KAL_G) * KAL_P  # 更新 协方差 = （1 - 卡尔曼增益） * 当前 估算协方差
    return KAL_Output


def KalmanFilter2(input):
    global KAL_P2
    global KAL_G2
    global KAL_Output2

    KAL_P2 = KAL_P2 + KAL_Q2  # 估算协方差方程：当前 估算协方差 = 上次更新 协方差 + 过程噪声协方差
    KAL_G2 = KAL_P2 / (KAL_P2 + KAL_R2)  # //卡尔曼增益方程：当前 卡尔曼增益 = 当前 估算协方差 / （当前 估算协方差 + 测量噪声协方差）
    # 更新最优值方程：当前 最优值 = 当前 估算值 + 卡尔曼增益 * （当前 测量值 - 当前 估算值）
    KAL_Output2 = KAL_Output2 + KAL_G2 * (input - KAL_Output2)  # 当前 估算值 = 上次 最优值
    KAL_P2 = (1 - KAL_G2) * KAL_P2  # 更新 协方差 = （1 - 卡尔曼增益） * 当前 估算协方差
    return KAL_Output2


def ips200_display():

    lcd.str16(0, 0, "Pitch={:f}.".format(round(Imu.Pitch, 3)), 0x001F)
    lcd.str16(0, 16, "Yaw={:f}.".format(Imu.Yaw), 0x001F)
    lcd.str16(0, 150, "med_speed{:>6d}, turn{:>6f}.".format(med_speed, turn), 0x001F)
    lcd.str16(0, 285, f"motor1: {motor1:.2f},motor2:{motor2:.2f}", 0x001F)
    lcd.str16(0, 223, "g = {:>6f}, {:>6f}, {:>6f}.".format(Imu.gyro_x, Imu.gyro_y, Imu.gyro_z), 0x001F)
    lcd.str12(0, 243, "lline2{:>6d}, rline2{:>6d}, zhong2{:>6d}.".format(lline2, rline2, zhong2), 0x001F)
    lcd.str12(0, 263, "lline{:>6d}, rline{:>6d}, zhong{:>6d}.".format(lline, rline, zhong), 0x001F)
    lcd.str16(0, 209, "enc ={:>6f}, {:>6f}\r\n".format(Encoders.KAL_templ_pluse, Encoders.KAL_tempr_pluse), 0x001F)

# 转向
def control_turn(zhong2):
    errt = (zhong2 - 64)
    turn_kp = a * abs(errt) + Kpc
    turn = pid_turn(64, zhong2, turn_kp, turn_ki, turn_kd) + Imu.gyro_z * turn_kd2

    return turn

# 角速度环
def angle_speed1(med_gyro, cur_gyro):
    motor = pid_position_1(med_gyro, cur_gyro, angle_kp, angle_ki, angle_kd)

    motor = limit(motor, -4000, 4000)
    return (motor)

# 角度环
def angle(med_roll_angle, cur_roll_angle):
    global angle_1

    angle_1 = pid_position_2(med_roll_angle, cur_roll_angle, roll_angle_Kp, roll_angle_Ki, roll_angle_Kd)
    return (angle_1)

# 速度环
def speed(med_speed, cur_speed):
    global speed_1
    speed_1 = pid_position_3(med_speed, cur_speed, speed_Kp, speed_Ki, speed_Kd)
    return speed_1


road = 0
fix = 0
circle = 0
zebra2 = 0
banma_slow = 0
stop_flag = 0

# 回调函数1
def time_pit_handler(time):
    global ticker_flag, ticker_count, speed_1, angle_1, motor1, motor2  # 需要注意的是这里得使用 global 修饰全局属性
    ticker_flag = True
    ticker_count = (ticker_count + 1) if (ticker_count < 10) else (1)  # 计数标注

    if ticker_count % 1 == 0:  # 角速度 1ms 执行一次
        Imu963()  # 陀螺仪解算
        #print("{:>6f}, {:>6f}, {:>6f}\n".format(Imu.Pitch, Imu.Roll, Imu.Yaw))  # 测试用，配置参数后通过vofa 串口打印
        # print("{:>6f}, {:>6f}, {:>6f}\n".format(Imu.gyro_x, Imu.gyro_y, Imu.gyro_z))  # 测试用，配置参数后通过vofa 串口打印
        motor1 = angle_speed1(angle_1, Imu.gyro_x)
        motor2 = angle_speed1(angle_1, Imu.gyro_x)

        motor1 = limit(motor1, -2000, 2000)  # 限幅
        motor2 = limit(motor2, -2000, 2000)

        motor_l.duty(-motor1)  # 输出
        motor_r.duty(-motor2)

    if ticker_count % 5 == 0:  # 角度 5ms 执行一次
        angle_1 = angle(med_roll_angle - 0, -Imu.Roll) #确定俯仰角的值

    if ticker_count % 10 == 0:  # 速度 10ms 执行一次
        encl_data = encoder_l.get()
        encr_data = encoder_r.get()

        speed_1 = speed(0, (Encoders.KAL_templ_pluse + Encoders.KAL_tempr_pluse) / 2)

# 回调函数2
def time_pit2_handler(time):
    global ticker_flag2, ticker_count2  # 需要注意的是这里得使用 global 修饰全局属性
    ticker_flag2 = True  # 否则它会新建一个局部变量
    global KAL_templ_pluse
    global KAL_tempr_pluse

    Encoders.KAL_templ_pluse = KalmanFilter(encoder_l.get())
    Encoders.KAL_tempr_pluse = KalmanFilter2(encoder_r.get())



pit1 = ticker(1)
pit2 = ticker(2)
pit3 = ticker(3)
pit1.capture_list(imu)
pit2.capture_list(ccd)
pit3.capture_list(encoder_l, encoder_r)
# 关联 Python 回调函数
pit1.callback(time_pit_handler)
pit2.callback(time_pit2_handler)

# 启动 ticker 实例 参数是触发周期 单位是毫秒
Imu963ra_Init()
pit1.start(1)
pit2.start(6)
pit3.start(10)

# 主循环
while True:

    if ticker_flag:
        ccd_data1 = ccd.get(0)
        ccd_data2 = ccd.get(1)
        # ccd 处理代码在这调用

        ticker_flag = False
    #ips200_display()

    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if end_switch.value() != end_state:
        pit1.stop()
        pit2.stop()
        pit3.stop()
        print("Ticker stop.")
        break

    gc.collect()

