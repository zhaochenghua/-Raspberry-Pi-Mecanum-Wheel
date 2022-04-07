import numpy as np
import cv2
from camera import Camera
from motor import Motor
import smbus

def draw_direction(img, lx, ly, nx, ny):
    # 根据上一位置与当前位置计算移动方向并绘制箭头
    dx = nx - lx
    dy = ny - ly
    if abs(dx) < 4 and abs(dy) < 4:
        dx = 0
        dy = 0
    else:
        r = (dx ** 2 + dy ** 2) ** 0.5
        dx = int(dx / r * 40)
        dy = int(dy / r * 40)
        # print(dx, dy)
    cv2.arrowedLine(img, (60, 100), (60 + dx, 100 + dy), (0, 255, 0), 2)


def pidMoveCtrl():
    global turn_speed, FB_speed
    if abs(targetPos_y - lastPos_y) < 30 and abs(targetPos_x - lastPos_x) < 30:
        FB_speed = int((targetPos_y ** 2) / 960 - 11 * targetPos_y / 24 + 40)
        if FB_speed > 30:
            FB_speed = 30
        turn_speed = int(0.2 * targetPos_x - 32)
        if turn_speed > 20:
            turn_speed = 20
        m.move(0, FB_speed, turn_speed)  # (LR_speed, FB_speed, turn_speed)


if __name__ == '__main__':
    # lower = np.array([10, 110, 110])  # 适用于橙黄色乒乓球
    # upper = np.array([20, 255, 255])
    # lower = np.array([85, 110, 67])  # 适用于蓝色乒乓球
    # upper = np.array([108, 255, 255])
    lower = np.array([20, 110, 67])  # 适用于黄色乒乓球
    upper = np.array([50, 255, 255])
    targetPos_x = 0
    targetPos_y = 0
    lastPos_x = 0
    lastPos_y = 0
    camera = Camera(0)
    camera.open()
    m = Motor()
    FB_speed = turn_speed = 0
    bus = smbus.SMBus(1)
    pos = [255, 90, 90, 90, 90, 90, 90]#1-6号舵机初始值
    moveSpeed = 10
    while True:
        ret, img = camera.getframe()
        findBall = False
        if ret:
            imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            imgMask = cv2.inRange(imgHsv, lower, upper)  # 获取遮罩
            imgOutput = cv2.bitwise_and(img, img, mask=imgMask)
            im, contours, hierarchy = cv2.findContours(imgMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 查找轮廓
            # 下面的代码查找包围框，并绘制
            x, y, w, h = 0, 0, 0, 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                # print(area)
                if  10< area < 1400:
                    findBall = True
                    x, y, w, h = cv2.boundingRect(cnt)
                    lastPos_x = targetPos_x
                    lastPos_y = targetPos_y
                    targetPos_x = int(x + w / 2)
                    targetPos_y = int(y + h / 2)
                    pidMoveCtrl()  # pid 电机控制，放在图像显示前，提高实时性。
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(img, (targetPos_x, targetPos_y), 2, (0, 255, 0), 4)
                # 坐标与箭头显示（图像内的）
                cv2.putText(img, "({:0<2d}, {:0<2d})".format(targetPos_x, targetPos_y), (20, 30),
                            cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)  # 文字
                cv2.putText(img, "({:0<2d}, {:0<2d})".format(FB_speed, turn_speed), (20, 200),
                            cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)  # 文字
                draw_direction(img, lastPos_x, lastPos_y, targetPos_x, targetPos_y)
            cv2.imshow('img', img)
            print(targetPos_x, targetPos_y)
            cv2.waitKey(10)
        else:
            print("read faild")
        if not findBall:
            if (targetPos_y>30 and targetPos_x >220):
                m.move(0, 0, 15)
            elif (targetPos_y>30 and targetPos_x <100):
                m.move(0, 0, -15)
            else:
                m.MX_motorUnlockAll()
        if cv2.waitKey(1) & 0xFF == ord('q'):  # 长按Q退出
            print('quit')
            break
    m.MX_motorUnlockAll()
    camera.close()
    cv2.destroyAllWindows()
