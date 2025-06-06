from machine import Pin

import gc
import time

# 上电启动时间延时
time.sleep_ms(50)
# 选择学习板上的一号拨码开关作为启动选择开关
boot_select = Pin('B28', Pin.IN, pull=Pin.PULL_UP_47K, value = True)

# 如果拨码开关打开 对应引脚拉低 就启动用户文件
if boot_select.value() == 0:
    try:
        os.chdir("/flash")
        execfile("user_main.py")
    except:
        print("File not found.")
