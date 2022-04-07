from motor import Motor
import math
from time import sleep


class 画图:
    def __init__(self) -> None:
        self.angle = 0
        self.m = Motor()

    def 画圆(self, r):
        angle = 0
        while True:
            self.m.move_to(x=math.sin(angle) * r, y=math.cos(angle) * r)
            angle += 0.1
            if angle > math.pi * 2:
                self.m.MX_motorLockAll()
                break
    # def 画直线(self,时间,速度,方向角):
    #     t = 0
    #     while (t < 时间):
    #         self.m.move(int(时间*速度 * math.cos(方向角*math.pi/180)), int(时间*速度 * math.sin(方向角*math.pi/180)), 0)
    #         sleep(0.1)
    #         t= t + 0.1
    #     self.m.MX_motorUnlockAll()
