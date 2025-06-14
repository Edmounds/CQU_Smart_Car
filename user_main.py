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
roll_angle2 = 0  # 添加全局变量，用于存储备用Roll计算结果
end_switch = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K, value=True)
end_state = end_switch.value()

# 定义片选引脚
fenmingqi = Pin('D24', Pin.OUT, pull=Pin.PULL_UP_47K, value=0)

# 定义片选引脚 拉高拉低一次 CS 片选确保屏幕通信时序正常
cs = Pin('B29' , Pin.OUT, value=True)
cs.high()
cs.low()

# 定义控制引脚
rst = Pin('B31', Pin.OUT, value=True)
dc  = Pin('B5' , Pin.OUT, value=True)
blk = Pin('C21', Pin.OUT, value=True)

# 构造接口 用于构建一个 LCD_Drv 对象
#   LCD_Drv(SPI_INDEX, BAUDRATE, DC_PIN, RST_PIN, LCD_TYPE)
#   SPI_INDEX   接口索引    |   必要参数 关键字输入 选择屏幕所用的 SPI 接口索引
#   BAUDRATE    通信速率    |   必要参数 关键字输入 SPI 的通信速率 最高 60MHz
#   DC_PIN      命令引脚    |   必要参数 关键字输入 一个 Pin 实例
#   RST_PIN     复位引脚    |   必要参数 关键字输入 一个 Pin 实例
#   LCD_TYPE    屏幕类型    |   必要参数 关键字输入 目前仅支持 LCD_Drv.LCD200_TYPE
drv = LCD_Drv(SPI_INDEX=2, BAUDRATE=60000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)

# 构造接口 用于构建一个 LCD 对象
#   LCD(LCD_Drv)
#   LCD_Drv     接口对象    |   必要参数 LCD_Drv 对象
lcd = LCD(drv)

# 修改 LCD 的前景色与背景色
#   LCD.color(pcolor, bgcolor)
#   pcolor      前景色     |   必要参数 RGB565 格式
#   bgcolor     背景色     |   必要参数 RGB565 格式
lcd.color(0xFFFF, 0x0000)

# 修改 LCD 的显示方向
#   LCD.mode(dir)
#   dir         显示方向    |   必要参数 [0:竖屏,1:横屏,2:竖屏180旋转,3:横屏180旋转]
lcd.mode(0)

# 清屏 不传入参数就使用当前的 背景色 清屏
#   LCD.clear([color])
#   color       颜色数值    |   非必要参数 RGB565 格式 输入参数则更新背景色并清屏
lcd.clear(0x0000)
# 电机初始化
motor_l = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_PWM_C29, 13000, duty=0, invert=True)
motor_r = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D6_DIR_D7, 13000, duty=0, invert=True)
motor_dir = 1
motor_duty = 0
motor_duty_max = 1000

#wifi模块初始化
wifi = WIFI_SPI("Muelsyse", "88888888", WIFI_SPI.TCP_CONNECT, "192.168.8.143", "8086")

turn = 0
led = Pin('C4', Pin.OUT, pull=Pin.PULL_UP_47K, value=True)
# 编码器初始化
encoder_l = encoder("C2", "C3")
encoder_r = encoder("C0", "C1", True)


# 调用 TSL1401 模块获取 CCD 实例
# 参数是采集周期 调用多少次 capture 更新一次数据
# 默认参数为 1 调整这个参数相当于调整曝光时间倍数
ccd = TSL1401(10)
# 调整 CCD 的采样精度为 12bit 数值范围 [0, 4095]
ccd.set_resolution(TSL1401.RES_12BIT)

# 陀螺仪初始化
imu = IMU963RX()
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
# 添加显示所需变量的初始化
lline = 0
rline = 0
zhong = 0
lline2 = 0
rline2 = 0
zhong2 = 0
med_speed = 0


##############################################UART
# 将原先的UART通信改为使用WIFI_SPI
# uart2 = UART(2)
# uart2.init(115200)


distance = 0
id_number = 0  # 初始化变量m为整数

# 串级参数
# ////////////角速度//////////////////////
angle_kp = -200.0  # 原始值-1270.0, 进一步减小负反馈系数
angle_ki = 0.0  # 原始值-20.00, 积分项先设为0
angle_kd = 0.0  # 原始值-180.0, 微分项先设为0
# ////////////角度//////////////////////
roll_angle_Kp = 0.2  # 增大P参数，从0.02增大到0.2，增强对角度的响应
roll_angle_Ki = 0.0  # 原始值0.000001, 积分项先设为0
roll_angle_Kd = 0.0  # 原始值0.1011, 微分项先设为0
# ////////////速度//////////////////////
speed_Kp = 0.0  # 原始值0.065, 速度环先关闭
speed_Ki = 0.0  # 原始值0.00012, 速度环先关闭
speed_Kd = 0.0
angle_1 = 0
speed_1 = 0
counts = 0
motor1 = 0
motor2 = 0
# 机械中值设置为备用Roll计算的实际值，这样两者接近，方便测试
med_roll_angle = 114 
med_speed1 = 0

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


#############################################################
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
        self.q0 = 0.7071  # 初始化为表示45度旋转的四元数
        self.q1 = 0.0
        self.q2 = 0.7071
        self.q3 = 0.0


Q_info = QInfo()
delta_T = 0.001  # 假设采样周期为5ms
I_ex, I_ey, I_ez = 0.0, 0.0, 0.0  # 积分误差


def invSqrt(x):
    # 使用正确的平方根倒数计算
    if x <= 0:
        return 1.0  # 防止除以0错误
    return 1.0 / math.sqrt(x)


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
    global imu_data, max_gyro_x, roll_angle2
    
    # 获取IMU数据并打印原始值，帮助诊断
    imu_data = imu.get()
    
    # 记录原始IMU数据，用于调试显示
    raw_acc_x = imu_data[0]
    raw_acc_y = imu_data[1] 
    raw_acc_z = imu_data[2]
    raw_gyro_x = imu_data[3]
    raw_gyro_y = imu_data[4]
    raw_gyro_z = imu_data[5]
    
    # 对异常值进行过滤
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
    
    # 直接计算Roll角度，使用加速度计数据（主要方案）
    # 由于四元数计算有问题，改为直接使用加速度计算倾角
    # 尝试使用不同的轴组合计算角度

    # 角度计算方法1: Y/Z轴 (传统Roll角计算)
    if abs(Imu.acc_z) > 0.01:
        roll_angle1 = math.atan2(Imu.acc_y, Imu.acc_z) * 180 / PI
    else:
        roll_angle1 = 90.0 if Imu.acc_y > 0 else -90.0
    
    # 角度计算方法2: X/Z轴 (替代Roll角计算)
    if abs(Imu.acc_z) > 0.01:
        roll_angle2 = math.atan2(Imu.acc_x, Imu.acc_z) * 180 / PI
    else:
        roll_angle2 = 90.0 if Imu.acc_x > 0 else -90.0
    
    # 角度计算方法3: Z/Y轴 (反向计算)
    if abs(Imu.acc_y) > 0.01:
        roll_angle3 = math.atan2(Imu.acc_z, Imu.acc_y) * 180 / PI
    else:
        roll_angle3 = 90.0 if Imu.acc_z > 0 else -90.0

    # 使用方法3，因为这可能与您的IMU安装方向更匹配
    # 也可以尝试其他方法，比如 roll_angle2，观察哪个能让车子在全范围内响应
    Imu.Roll = roll_angle3
    
    # 可能需要进行角度范围调整，确保角度能够穿过90度或180度边界
    # 例如，如果需要角度范围在0-180之间，可以这样调整:
    if Imu.Roll < 0:
        Imu.Roll += 180
    
    # 如果Roll值不变，尝试使用其他计算方式
    if ticker_count % 100 == 0:
        print("Roll angles: 1={:.2f}, 2={:.2f}, 3={:.2f}".format(roll_angle1, roll_angle2, roll_angle3))
        
    # 四元素调用 - 暂时不使用四元数计算结果
    IMU_AHRSupdate(Imu.gyro_x, Imu.gyro_y, Imu.gyro_z, Imu.acc_x, Imu.acc_y, Imu.acc_z)
    
    # 保存原始四元数计算结果用于比较
    quat_roll = Imu.Roll
        
    if abs(max_gyro_x) < abs(Imu.Pitch):
        max_gyro_x = Imu.Pitch

# 陀螺仪初始化
def Imu963ra_Init():
    global Filter_data
    global imu_data
    global roll_angle2
    
    # 先输出一些调试信息
    print("Starting IMU initialization...")
    
    Filter_data[0] = 0
    Filter_data[1] = 0
    Filter_data[2] = 0

    for i in range(0, 1000):
        imu_data = imu.get()
        Filter_data[0] += imu_data[3]
        Filter_data[1] += imu_data[4]
        Filter_data[2] += imu_data[5]
        if i % 100 == 0:  # 每100次采样打印一次数据
            print("Sample {}: gyro={},{},{}".format(i, imu_data[3], imu_data[4], imu_data[5]))
        time.sleep_ms(1)
    
    Filter_data[0] = float(Filter_data[0] / 1000)
    Filter_data[1] = float(Filter_data[1] / 1000)
    Filter_data[2] = float(Filter_data[2] / 1000)
    
    # 初始化roll_angle2的值
    imu_data = imu.get()
    # 计算初始的加速度值
    acc_x = float(imu_data[0]) / 4096
    acc_y = float(imu_data[1]) / 4096
    acc_z = float(imu_data[2]) / 4096
    
    # 计算初始的Roll角度，用于验证初始状态
    if abs(acc_z) > 0.01:
        roll_angle = math.atan2(acc_y, acc_z) * 180 / PI
    else:
        roll_angle = 90.0 if acc_y > 0 else -90.0
        
    if abs(acc_x) > 0.01 or abs(acc_z) > 0.01:
        roll_angle2 = math.atan2(acc_x, acc_z) * 180 / PI
    else:
        roll_angle2 = 90.0 if acc_x > 0 else -90.0
    
    print("IMU initialization complete. Filter_data=", Filter_data)
    print("Initial roll_angle={:.2f}, roll_angle2={:.2f}".format(roll_angle, roll_angle2))
    
    # 尝试手动找出合适的轴组合
    print("Testing all possible axis combinations for Roll calculation:")
    print("Y/Z: {:.2f}".format(math.atan2(acc_y, acc_z) * 180 / PI))
    print("X/Z: {:.2f}".format(math.atan2(acc_x, acc_z) * 180 / PI))
    print("X/Y: {:.2f}".format(math.atan2(acc_x, acc_y) * 180 / PI))
    print("Z/Y: {:.2f}".format(math.atan2(acc_z, acc_y) * 180 / PI))
    print("Z/X: {:.2f}".format(math.atan2(acc_z, acc_x) * 180 / PI))
    print("Y/X: {:.2f}".format(math.atan2(acc_y, acc_x) * 180 / PI))

# 四元数
def IMU_AHRSupdate(gx, gy, gz, ax, ay, az):
    global I_ex, I_ey, I_ez, last_yaw
    halfT = 0.5 * delta_T
    value1 = 0
    # 当前的机体坐标系上的重力单位向量
    vx, vy, vz = 0.0, 0.0, 0.0
    ex, ey, ez = 0.0, 0.0, 0.0
    
    # 计算重力在机体坐标系中的投影
    q0q0 = Q_info.q0 * Q_info.q0
    q0q1 = Q_info.q0 * Q_info.q1
    q0q2 = Q_info.q0 * Q_info.q2
    q0q3 = Q_info.q0 * Q_info.q3
    q1q1 = Q_info.q1 * Q_info.q1
    q1q3 = Q_info.q1 * Q_info.q3
    q2q2 = Q_info.q2 * Q_info.q2
    q2q3 = Q_info.q2 * Q_info.q3
    q3q3 = Q_info.q3 * Q_info.q3

    # 检测加速度是否接近零，防止归一化异常
    acc_norm = ax*ax + ay*ay + az*az
    if acc_norm < 0.01:  # 如果加速度几乎为零，跳过本次更新
        return
        
    # 对加速度数据进行归一化
    norm = invSqrt(acc_norm)
    ax *= norm
    ay *= norm
    az *= norm

    # 计算当前重力单位向量
    vx = 2 * (q1q3 - q0q2)
    vy = 2 * (q0q1 + q2q3)
    vz = q0q0 - q1q1 - q2q2 + q3q3

    # 计算加速度计测量的重力与姿态计算的重力间的差值
    ex = ay * vz - az * vy
    ey = az * vx - ax * vz
    ez = ax * vy - ay * vx

    # 用误差进行PI修正
    I_ex += delta_T * ex  # 积分误差
    I_ey += delta_T * ey
    I_ez += delta_T * ez
    
    # 限制积分范围，防止积分饱和
    I_ex = limit(I_ex, -0.1, 0.1)
    I_ey = limit(I_ey, -0.1, 0.1)
    I_ez = limit(I_ez, -0.1, 0.1)

    gx += Param.param_Kp * ex + Param.param_Ki * I_ex
    gy += Param.param_Kp * ey + Param.param_Ki * I_ey
    gz += Param.param_Kp * ez + Param.param_Ki * I_ez

    # 四元数微分方程，使用龙格库塔法更新四元数
    q0 = Q_info.q0
    q1 = Q_info.q1
    q2 = Q_info.q2
    q3 = Q_info.q3

    # 四元数更新公式
    Q_info.q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT
    Q_info.q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT
    Q_info.q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT
    Q_info.q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT

    # 归一化四元数，确保四元数的模为1
    norm = invSqrt(Q_info.q0**2 + Q_info.q1**2 + Q_info.q2**2 + Q_info.q3**2)
    if norm == 0:  # 防止除以零
        return
        
    Q_info.q0 *= norm
    Q_info.q1 *= norm
    Q_info.q2 *= norm
    Q_info.q3 *= norm

    # 计算欧拉角
    value1 = Limit(-2 * Q_info.q1 * Q_info.q3 + 2 * Q_info.q0 * Q_info.q2)
    Imu.Roll = round(math.asin(value1) * 180 / math.pi, 3)  # pitch
    Imu.Pitch = round(math.atan2(2 * Q_info.q2 * Q_info.q3 + 2 * Q_info.q0 * Q_info.q1,
                                 -2 * Q_info.q1**2 - 2 * Q_info.q2**2 + 1) * 180 / math.pi, 3)  # roll
    Imu.Yaw = round(math.atan2(2 * Q_info.q1 * Q_info.q2 + 2 * Q_info.q0 * Q_info.q3,
                               -2 * Q_info.q2**2 - 2 * Q_info.q3**2 + 1) * 180 / math.pi, 3)  # yaw
    
    # 打印调试信息
    if abs(Imu.Roll) < 0.001:  # 如果计算的Roll几乎为0
        print("Warning: Roll is near zero! q0={:.4f}, q1={:.4f}, q2={:.4f}, q3={:.4f}".format(
            Q_info.q0, Q_info.q1, Q_info.q2, Q_info.q3))
    
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
    lcd.str16(0, 0, "Roll={:f}, raw_roll2={:f}".format(round(Imu.Roll, 3), roll_angle2), 0x001F)
    lcd.str16(0, 16, "raw_gyro={:.1f},{:.1f},{:.1f}".format(imu_data[3], imu_data[4], imu_data[5]), 0x001F)
    lcd.str16(0, 32, "raw_acc={:.1f},{:.1f},{:.1f}".format(imu_data[0], imu_data[1], imu_data[2]), 0x001F)
    lcd.str16(0, 48, "gyro_x,y,z={:.3f},{:.3f},{:.3f}".format(Imu.gyro_x, Imu.gyro_y, Imu.gyro_z), 0x001F)
    lcd.str16(0, 64, "acc_x,y,z={:.3f},{:.3f},{:.3f}".format(Imu.acc_x, Imu.acc_y, Imu.acc_z), 0x001F)
    lcd.str16(0, 80, "med_roll={:f}, angle_1={:f}".format(med_roll_angle, angle_1), 0x001F)
    lcd.str16(0, 150, "med_speed={:>6d}, turn={:>6f}".format(med_speed, turn), 0x001F)
    lcd.str16(0, 209, "enc ={:>6f}, {:>6f}".format(Encoders.KAL_templ_pluse, Encoders.KAL_tempr_pluse), 0x001F)
    lcd.str16(0, 285, "motor1: {:.2f}, motor2:{:.2f}".format(motor1, motor2), 0x001F)
    
    # 发送数据到逐飞助手虚拟示波器显示
    wifi.send_oscilloscope(
        motor1, motor2, Imu.Roll, roll_angle2, 
        Encoders.KAL_templ_pluse, Encoders.KAL_tempr_pluse, turn, med_speed)

# 转向
def control_turn(zhong2):
    errt = (zhong2 - 64)
    turn_kp = a * abs(errt) + Kpc
    turn = pid_turn(64, zhong2, turn_kp, turn_ki, turn_kd) + Imu.gyro_z * turn_kd2

    return turn

# 角速度环
def angle_speed1(med_gyro, cur_gyro):
    motor = pid_position_1(med_gyro, cur_gyro, angle_kp, angle_ki, angle_kd)
    
    # 使用非线性调整，增加响应灵敏度
    if abs(motor) > 200:
        # 大偏差时增加输出增益
        motor = motor * 1.5
    
    motor = limit(motor, -4000, 4000)
    return (motor)

# 角度环
def angle(med_roll_angle, cur_roll_angle):
    global angle_1
    
    # 计算角度差值，考虑可能的角度环绕问题
    angle_error = med_roll_angle - cur_roll_angle
    
    # 如果错误太大，可能是穿过了0/360或+/-180的边界，尝试调整
    if angle_error > 180:  # 例如 med=350, cur=10 -> error=340 应该是 -20
        angle_error -= 360
    elif angle_error < -180:  # 例如 med=10, cur=350 -> error=-340 应该是 20
        angle_error += 360
        
    # 在距离平衡点较远时，使用更强的P增益
    adaptive_kp = roll_angle_Kp
    if abs(angle_error) > 15:
        # 角度偏差大于15度时，加大P增益给更强的响应
        adaptive_kp = roll_angle_Kp * 1.5
        
    # 如果error为正，车子应向一个方向调整，如果为负，则向另一方向调整
    # 打印错误值，帮助诊断
    if ticker_count % 50 == 0:
        print("Angle error: {:.2f}, adaptive_kp: {:.4f}".format(angle_error, adaptive_kp))
    
    # 使用错误值直接计算，而不是使用原始值
    angle_1 = pid_position_2(0, -angle_error, adaptive_kp, roll_angle_Ki, roll_angle_Kd)
    return angle_1

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
    global ticker_flag, ticker_count, speed_1, angle_1, motor1, motor2
    ticker_flag = True
    ticker_count = (ticker_count + 1) if (ticker_count < 10) else (1)

    if ticker_count % 1 == 0:  # 角速度 1ms 执行一次
        Imu963()  # 陀螺仪解算
        
        # 使用修改后的安全检查逻辑
        if abs(Imu.Roll - med_roll_angle) > 70:  # 扩大安全范围到70度
            # 角度偏差太大，但不要停止电机，而是限制输出
            if ticker_count % 100 == 0:
                print("WARNING: Large angle difference: {:.2f}".format(Imu.Roll - med_roll_angle))
                
        # 计算角速度环输出
        motor1 = angle_speed1(angle_1, Imu.gyro_x)
        motor2 = angle_speed1(angle_1, Imu.gyro_x)
        
        # 限幅
        motor1 = limit(motor1, -2000, 2000)
        motor2 = limit(motor2, -2000, 2000)
        
        # 电机死区补偿
        if abs(motor1) < 300 and abs(motor1) > 20:  # 增大死区补偿
            motor1 = 300 if motor1 > 0 else -300
        if abs(motor2) < 300 and abs(motor2) > 20:
            motor2 = 300 if motor2 > 0 else -300

        # 输出到电机 - 使用负号表示反向控制
        motor_l.duty(-motor1)
        motor_r.duty(-motor2)

        # 临时调试代码 - 每50ms打印一次IMU数据
        if ticker_count % 50 == 0:
            print("IMU: Roll={:.2f}, error={:.2f}, motor={:.0f},{:.0f}".format(
                Imu.Roll, med_roll_angle - Imu.Roll, motor1, motor2))

    if ticker_count % 5 == 0:  # 角度 5ms 执行一次
        # 角度环的目标值(med_roll_angle)需要减去速度环的输出(speed_1)
        angle_1 = angle(med_roll_angle - speed_1, Imu.Roll)
        
        # 打印角度环的值
        if ticker_count % 10 == 0:
            print("Angle loop: target={:.2f}, current={:.2f}, output={:.2f}".format(
                med_roll_angle - speed_1, Imu.Roll, angle_1))

    if ticker_count % 10 == 0:  # 速度 10ms 执行一次
        encl_data = encoder_l.get()
        encr_data = encoder_r.get()

        # 暂时关闭速度环
        speed_1 = 0  # 先注释掉速度控制，专注于平衡
        # speed_1 = speed(0, (Encoders.KAL_templ_pluse + Encoders.KAL_tempr_pluse) / 2)

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
        # 获取CCD数据
        ccd_data1 = ccd.get(0)  # 获取第一个CCD的数据
        ccd_data2 = ccd.get(1)  # 获取第二个CCD的数据
        
        # 发送CCD图像到逐飞助手显示
        wifi.send_ccd_image(WIFI_SPI.CCD1_2_BUFFER_INDEX)
        
        # 数据解析
        data_flag = wifi.data_analysis()
        for i in range(0,8):
            # 判断哪个通道有数据更新
            if (data_flag[i]):
                # 数据更新到缓冲
                data_wave = wifi.get_data(i)
                # 根据接收到的数据更新参数
                if i == 0:
                    angle_kp = data_wave
                elif i == 1:
                    angle_ki = data_wave
                elif i == 2:
                    angle_kd = data_wave
                elif i == 3:
                    roll_angle_Kp = data_wave
                elif i == 4:
                    roll_angle_Ki = data_wave
                elif i == 5:
                    roll_angle_Kd = data_wave
                elif i == 6:
                    med_roll_angle = data_wave
                elif i == 7:
                    med_speed = data_wave
        
        ticker_flag = False
    
    # 显示屏更新
    ips200_display()
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if end_switch.value() != end_state:
        pit1.stop()
        pit2.stop()
        pit3.stop()
        print("Ticker stop.")
        break

    gc.collect()


