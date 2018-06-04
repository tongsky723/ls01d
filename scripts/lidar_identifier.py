#!/usr/bin/python
# -*- coding: utf-8 -*-
# @Time    : 18-4-20 上午9:15
# @Author  : Yutong
# @Email   : 416178264@qq.com
# @Site    : 
# @File    : lidar_identifier.py
# @Software: PyCharm
# @Desc    :

import math
import time
import serial
from array import *

d2r = (2 * math.pi) / 360.0
r2d = 360.0 / (2 * math.pi)


if __name__ == '__main__':
    try:
        serial_baudrate = 230400
        serial_port = '/dev/laser'
        time_out = 1.5
        cmd_id = [0xA5, 0x7F]
        cmd_stop = [0xA5, 0x8F]
        cmdstr_start = array("B", cmd_id).tostring()
        cmdstr_stop = array("B", cmd_stop).tostring()

        try:
            device = serial.Serial(serial_port, serial_baudrate, timeout=time_out)
            time.sleep(0.2)
        except Exception as e:
            print("can not open port %s with reason %s" %(serial_port, e))
            exit()

        print("lidar is opened successfully!")

        device.write(cmdstr_start)
        try:
            data = device.read(9)
            # for i in range(9):
            #     print(data[i], ord(data[i]))

            if (len(data) == 9) and (data[0] == '\xA5'):
                model_num = ord(data[1]) * 256 + ord(data[2])
                print("lidar model is ls%s" %hex(model_num))
            else:
                print("lidar model is ls01c")
        except Exception as e:
            # print("can not read from serial device %s, error %s" %(data, e))
            print("lidar model is ls01c")


    except Exception as e:
        print("code failed with reason %s" %e)
        device.close()

