import math

import serial
import RPi.GPIO as GPIO
import time


class Motor:
    def __init__(self) -> None:
        self.ser = serial.Serial(
            port='/dev/ttyS0',  # Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        self.moveSpeed = 10

        # ############端口定义#############
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.MX_digitalIO_1 = 16  # //电机1通讯口,低电平有效
        self.MX_digitalIO_2 = 19  # //电机2通讯口,低电平有效
        self.MX_digitalIO_3 = 20  # //电机3通讯口,低电平有效
        self.MX_digitalIO_4 = 26  # //电机4通讯口,低电平有效

        # #######GPIO初始化为HIGH##########
        GPIO.setup(self.MX_digitalIO_1, GPIO.OUT)
        GPIO.setup(self.MX_digitalIO_2, GPIO.OUT)
        GPIO.setup(self.MX_digitalIO_3, GPIO.OUT)
        GPIO.setup(self.MX_digitalIO_4, GPIO.OUT)
        GPIO.output(self.MX_digitalIO_1, True)
        GPIO.output(self.MX_digitalIO_2, True)
        GPIO.output(self.MX_digitalIO_3, True)
        GPIO.output(self.MX_digitalIO_4, True)

    def MX_motorControl(self, MX_motorControl_port, MX_motorControl_function, MX_motorControl_content1,
                        MX_motorControl_content2):
        # ######电机方向设置#############
        if (MX_motorControl_port == self.MX_digitalIO_1 or MX_motorControl_port == self.MX_digitalIO_3):  # 左侧电机反转
            MX_motorControl_content1 = -MX_motorControl_content1
        # #######电机端口易用化###########
        if (MX_motorControl_port == 1):
            MX_motorControl_port = self.MX_digitalIO_1
        elif (MX_motorControl_port == 2):
            MX_motorControl_port = self.MX_digitalIO_2
        elif (MX_motorControl_port == 3):
            MX_motorControl_port = self.MX_digitalIO_3
        elif (MX_motorControl_port == 4):
            MX_motorControl_port = self.MX_digitalIO_4
        # ########处理负值##############
        if (MX_motorControl_content1 < 0):  # 负值要取反加一
            MX_motorControl_content1 = (~(-MX_motorControl_content1) & 0xFF) + 1
        # ########开始发送操作##########
        GPIO.output(MX_motorControl_port, False)  # //通讯标志位置低，通知开始发送
        serial_data = bytes(
            [165, MX_motorControl_function, MX_motorControl_content1, (MX_motorControl_content2 >> 24) & 0xFF, \
             (MX_motorControl_content2 >> 16) & 0xFF, (MX_motorControl_content2 >> 8) & 0xFF,
             MX_motorControl_content2 & 0xFF, \
             MX_motorControl_function ^ MX_motorControl_content1 ^ (MX_motorControl_content2 >> 24) & 0xFF ^ (
                     MX_motorControl_content2 >> 16) & 0xFF ^ (
                     MX_motorControl_content2 >> 8) & 0xFF ^ MX_motorControl_content2 & 0xFF])
        self.ser.write(serial_data)  # 发送数据准备:起始位0xa5,功能位，内容位1，内容位2（4部分）,校验位###
        time.sleep(0.001)  # 等待1ms
        GPIO.output(MX_motorControl_port, True)  # 通讯标志位置高，通知结束发送

    def MX_motorUnlockAll(self):
        self.MX_motorControl(self.MX_digitalIO_1, 22, 0, 0)
        self.MX_motorControl(self.MX_digitalIO_2, 22, 0, 0)
        self.MX_motorControl(self.MX_digitalIO_3, 22, 0, 0)
        self.MX_motorControl(self.MX_digitalIO_4, 22, 0, 0)

    def MX_motorLockAll(self):
        self.MX_motorControl(self.MX_digitalIO_1, 20, 0, 0)
        self.MX_motorControl(self.MX_digitalIO_2, 20, 0, 0)
        self.MX_motorControl(self.MX_digitalIO_3, 20, 0, 0)
        self.MX_motorControl(self.MX_digitalIO_4, 20, 0, 0)

    def MX_motorAngle(self, MX_motorAngle_port):
        self.MX_motorControl(MX_motorAngle_port, 23, 1, 0)
        time.sleep(0.01)
        MX_receive = self.ser.read(self.ser.inWaiting())
        if (MX_receive[5] == MX_receive[1] ^ MX_receive[2] ^ MX_receive[3] ^ MX_receive[4]):
            MX_motorAngleN = 0
            MX_motorAngleN = MX_motorAngleN << 8 | MX_receive[1]
            MX_motorAngleN = MX_motorAngleN << 8 | MX_receive[2]
            MX_motorAngleN = MX_motorAngleN << 8 | MX_receive[3]
            MX_motorAngleN = MX_motorAngleN << 8 | MX_receive[4]
        return MX_motorAngleN

    def move(self, LR, FB, turn):
        move_sum = abs(FB) + abs(LR) + abs(turn)
        if move_sum >= 100:
            k = 100 / move_sum
            FB = FB * k
            LR = LR * k
            turn = turn * k
        self.MX_motorControl(self.MX_digitalIO_1, 10, int(FB + LR * 1 + turn*1), 0)
        self.MX_motorControl(self.MX_digitalIO_2, 10, int(FB + LR * -1 + turn * -1), 0)
        self.MX_motorControl(self.MX_digitalIO_3, 10, int(FB + LR * -1 + turn*1), 0)
        self.MX_motorControl(self.MX_digitalIO_4, 10, int(FB + LR * 1 + turn * -1), 0)

    def move_to(self, x, y):
        t = 0
        distance = math.sqrt(x ** 2 + y ** 2)
        time_spent = distance / self.moveSpeed
        lr_speed = int(x * self.moveSpeed / distance)
        fb_speed = int(y * self.moveSpeed / distance)
        while t < time_spent:
            self.move(lr_speed, fb_speed, 0)
            time.sleep(0.1)
            t = t + 0.1


