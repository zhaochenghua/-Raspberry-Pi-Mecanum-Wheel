import smbus
import time

bus = smbus.SMBus(1)
add = 0x08
pos = [255, 0, 0, 0, 0, 0, 0]  # 定义六个舵机,255为数据头


def run_servo():
    for x in pos:
        bus.write_byte(add, x)  # 向地址8发送数据
        time.sleep(0.002)


class MyI2Cservo():
    def __init__(self, channel, min_theta, max_theta, init_theta=0):
        """
        构造函数：
            channel: 舵机信号线所连接的端口
            min_theta: 舵机转动的最小角度
            max_theta: 舵机转动的最大角度
            init_theta: 舵机的初始角度
        """

        self.channel = channel
        if min_theta < 0 or min_theta > 180:
            self.min_theta = 0
        else:
            self.min_theta = min_theta

        if max_theta < 0 or max_theta > 180:
            self.max_theta = 180
        else:
            self.max_theta = max_theta

        if init_theta < min_theta or init_theta > max_theta:
            self.init_theta = (self.min_theta + self.max_theta) / 2
        else:
            self.init_theta = init_theta  # 初始角度

    def setup(self):
        """
        servo180初始化
        """
        pos[self.channel] = int(self.init_theta)
        run_servo()

    def turn_to(self, angle):
        if angle > self.max_theta:
            angle = self.max_theta
        elif angle < self.min_theta:
            angle = self.min_theta
        pos[self.channel] = int(angle)
        run_servo()

class MyI2Cservo270():
    def __init__(self, channel, min_theta, max_theta, init_theta=0):
        """
        构造函数：
            channel: 舵机信号线所连接的端口
            min_theta: 舵机转动的最小角度
            max_theta: 舵机转动的最大角度
            init_theta: 舵机的初始角度
        """

        self.channel = channel
        if min_theta < 0 or min_theta > 270:
            self.min_theta = 0
        else:
            self.min_theta = min_theta

        if max_theta < 0 or max_theta > 270:
            self.max_theta = 270
        else:
            self.max_theta = max_theta

        if init_theta < min_theta or init_theta > max_theta:
            self.init_theta = (self.min_theta + self.max_theta) / 2
        else:
            self.init_theta = init_theta  # 初始角度

    def setup(self):
        """
        servo270初始化
        """
        pos[self.channel] = int(self.init_theta)
        run_servo()

    def turn_to(self, angle):
        if angle > self.max_theta:
            angle = self.max_theta
        elif angle < self.min_theta:
            angle = self.min_theta
        pos[self.channel] = int(angle * 180 / 270)
        run_servo()