import RPi.GPIO as GPIO
import time

#http://www.4k8k.xyz/article/chengyuweng7838/100996253
# 这个类表示单个的SG90舵机模块
class Rotation:
    frequency = 50  # 脉冲频率(Hz)
    delta_theta = 0.2  # 步进转动间隔(度)
    min_delay = 0.0006  # 转动delta_theta的理论耗时(s)
    max_delay = 0.4  # 从0转到180的耗时(s)

    def __init__(self, channel, min_theta, max_theta, init_theta=0):
        """
        构造函数：
            channel: 舵机信号线所连接的树莓派引脚编号（BCM编码）
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

        # 计算最小角度、最大角度的占空比
        self.min_dutycycle = 2.5 + self.min_theta * 10 / 180
        self.max_dutycycle = 2.5 + self.max_theta * 10 / 180

    def setup(self):
        """
        初始化
        """
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.channel, GPIO.OUT)
        self.pwm = GPIO.PWM(self.channel, Rotation.frequency)  # PWM
        self.dutycycle = 2.5 + self.init_theta * 10 / 180  # 脉冲占空比的初始值
        self.pwm.start(self.dutycycle)  # 让舵机转到初始位置
        time.sleep(Rotation.max_delay)

    def positiveRotation(self):
        """
        正相步进转动，每次调用只转动delta_theta度
        """
        self.dutycycle = self.dutycycle + Rotation.delta_theta * 10 / 180
        if self.dutycycle > self.max_dutycycle:
            self.dutycycle = self.max_dutycycle
        self.pwm.ChangeDutyCycle(self.dutycycle)
        time.sleep(Rotation.min_delay)

    def reverseRotation(self):
        """
        反相转动，每次调用只转动delta_theta度
        """
        self.dutycycle = self.dutycycle - Rotation.delta_theta * 10 / 180
        if self.dutycycle < self.min_dutycycle:
            self.dutycycle = self.min_dutycycle
        self.pwm.ChangeDutyCycle(self.dutycycle)
        time.sleep(Rotation.min_delay)

    def specifyRotation(self, theta):
        """
        转动到指定的角度
        """
        if theta < 0 or theta > 180:
            return
        self.dutycycle = 2.5 + theta * 10 / 180
        self.pwm.ChangeDutyCycle(self.dutycycle)
        time.sleep(Rotation.max_delay)

    def cleanup(self):
        self.pwm.stop()
        time.sleep(Rotation.min_delay)
        GPIO.cleanup()