from machine import *

from smartcar import *
from seekfree import *
from display import *

import gc
import time

def lcd_init():
    # 核心板上 C4 是 LED

    # 调用 machine 库的 Pin 类实例化一个引脚对象
    # 配置参数为 引脚名称 引脚方向 模式配置 默认电平
    # 详细内容参考 固件接口说明
    switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K, value = True)

    state2  = switch2.value()

    # 定义片选引脚
    cs = Pin('B29' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
    # 拉高拉低一次 CS 片选确保屏幕通信时序正常
    cs.high()
    cs.low()
    # 定义控制引脚
    rst = Pin('B31', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
    dc  = Pin('B5' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
    blk = Pin('C21', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
    # 新建 LCD 驱动实例 这里的索引范围与 SPI 示例一致 当前仅支持 IPS200
    drv = LCD_Drv(SPI_INDEX=2, BAUDRATE=60000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
    # 新建 LCD 实例
    lcd = LCD(drv)
    # color 接口设置屏幕显示颜色 [前景色,背景色]
    lcd.color(0xFFFF, 0x0000)
    # mode 接口设置屏幕显示模式 [0:竖屏,1:横屏,2:竖屏180旋转,3:横屏180旋转]
    lcd.mode(2)
    # 清屏 不传入参数就使用当前的 背景色 清屏
    # 传入 RGB565 格式参数会直接把传入的颜色设置为背景色 然后清屏
    lcd.clear(0b0000011111100000)
    return lcd

####################################主函数引脚分配#######################################
# 核心板上 C4 是 LED
led1 = Pin('C4' , Pin.OUT, pull = Pin.PULL_UP_47K, value = True)
# 选择学习板上的二号拨码开关作为退出选择开关
end_switch = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K, value = True)
#######################################
 


####################################中断器初始化#########################################
ticker_flag = False
ticker_count = 0
runtime_count = 0

# 定义一个回调函数 需要一个参数 这个参数就是 ticker 实例自身
def time_pit_handler(time):
    global ticker_flag  # 需要注意的是这里得使用 global 修饰全局属性
    global ticker_count
    ticker_flag = True  # 否则它会新建一个局部变量
    ticker_count = (ticker_count + 1) if (ticker_count < 100) else (1)

# 实例化 PIT ticker 模块 参数为编号 [0-3] 最多四个
pit1 = ticker(1)
pit2 = ticker(2)
pit3 = ticker(3)
#######################################


################################imu############################
imu = IMU963RA()


# 通过 capture 接口更新数据 但在这个例程中被 ticker 模块接管了
# imu.capture()
# 关联采集接口 最少一个 最多八个 (imu, ccd, key...)
# 可关联 smartcar 的 ADC_Group_x 与 encoder_x
# 可关联 seekfree 的  IMU660RA, IMU963RA, KEY_HANDLER 和 TSL1401
pit1.capture_list(imu)
# 关联 Python 回调函数
pit1.callback(time_pit_handler)
# 启动 ticker 实例 参数是触发周期 单位是毫秒
pit1.start(10)

###############################CCD##################################
ccd = TSL1401(10)
# 调整 CCD 的采样精度为 12bit
ccd.set_resolution(TSL1401.RES_12BIT)
pit2.capture_list(ccd)
pit2.callback(time_pit_handler)
pit2.start(10)

lcd = lcd_init()

encoderR = encoder("C2" , "C3" , True)
encoderL = encoder("C0" , "C1")
pit3.capture_list(encoderL, encoderR)
pit3.callback(time_pit_handler)
pit3.start(10)
####################################主函数执行###########################################

while True:
#   time.sleep_ms(100)
#   motor.Motor_0ut(duty_left=motor.L_SpeedControl, duty_right=motor.R_SpeedControl)
#   motor.Motor_0ut(1000, 1000)
#   UI.UI_process()#ips200 ui显示，占用资源过多，调试时可以放主函数，测速放100ms中断器中
    if (ticker_flag and ticker_count % 20 == 0):
        # 翻转 C4 LED 电平
        led1.toggle()
      
        # 通过 get 接口读取数据
        imu_data = imu.get()
        print("acc = {:>6d}, {:>6d}, {:>6d}.".format(imu_data[0], imu_data[1], imu_data[2]))
        print("gyro = {:>6d}, {:>6d}, {:>6d}.".format(imu_data[3], imu_data[4], imu_data[5]))
        print("mag = {:>6d}, {:>6d}, {:>6d}.".format(imu_data[6], imu_data[7], imu_data[8]))
      
        # 通过 get 接口读取数据 参数 [0,1,2,3] 对应学习板上 CCD1/2/3/4 接口
        for ccd_ind in range(2):
            ccd_data = ccd.get(ccd_ind)
            #print(ccd_data)
            for ind, val in enumerate(ccd_data[8:]):
                R = round(val * 31 / 4095)
                G = round(val * 63 / 4095)
                B = round(val * 31 / 4095)
                color = (R << 11) | (G << 5) | B
                lcd.line(ind * 2, 64 * ccd_ind + 16, ind * 2, 64 * ccd_ind + 32, color=color, thick=2)
        
        lcd.str32(0, 160, "L = %d, R = %d" % (encoderL.get(), encoderR.get()), 0b1111100000011111)

        ticker_flag = False
        runtime_count = runtime_count + 1
        if(0 == runtime_count % 100):
          print("runtime_count = {:>6d}.".format(runtime_count))
#######################################
    gc.collect()
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if end_switch.value() == 0:
        break

