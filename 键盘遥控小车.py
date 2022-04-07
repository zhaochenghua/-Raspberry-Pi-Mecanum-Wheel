from motor import Motor
import time
import pygame
import sys
import smbus

bus = smbus.SMBus(1)
pos = [255, 90, 90, 90, 90, 90, 90]
moveSpeed = 10


def servo270(port, angle):
    pos[port] = int(angle * 180 / 270)


def runservo():
    for x in pos:
        bus.write_byte(0x08, x)  # 向地址8发送数据
        time.sleep(0.002)


def keyboardevent():
    # 图像移动 KEYDOWN 键盘按下事件
    # 通过 key 属性对应按键
    global FB_speed, LR_speed, turn_speed, flag, runstate, moveSpeed
    if event.type == pygame.KEYDOWN:
        if event.key == pygame.K_ESCAPE:
            runstate = False
        if event.key == pygame.K_1 or event.key == pygame.K_KP1:
            m.moveSpeed = 10
        if event.key == pygame.K_2 or event.key == pygame.K_KP2:
            m.moveSpeed = 20
        if event.key == pygame.K_3 or event.key == pygame.K_KP3:
            m.moveSpeed = 30
        if event.key == pygame.K_w:
            site[1] -= 8
            FB_speed = FB_speed + m.moveSpeed
        if event.key == pygame.K_s:
            site[1] += 8
            FB_speed = FB_speed - m.moveSpeed
        if event.key == pygame.K_a:
            site[0] -= 8
            LR_speed = LR_speed - m.moveSpeed
        if event.key == pygame.K_d:
            site[0] += 8
            LR_speed = LR_speed + m.moveSpeed
        if event.key == pygame.K_LEFT:
            turn_speed = turn_speed - m.moveSpeed
        if event.key == pygame.K_RIGHT:
            turn_speed = turn_speed + m.moveSpeed
        # if event.key == pygame.K_UP:
        #     servo270(1, 170)
        #     runservo()
        # if event.key == pygame.K_DOWN:
        #     servo270(1, 35)
        #     runservo()
        if event.key == pygame.K_RETURN:
            if flag == 0:
                servo270(2, 200)
                runservo()
                flag = 1
            elif flag == 1:
                servo270(2, 90)
                runservo()
                flag = 0
    if event.type == pygame.KEYUP:
        if event.key == pygame.K_w:
            site[1] += 8
            FB_speed = FB_speed - m.moveSpeed
        if event.key == pygame.K_s:
            site[1] -= 8
            FB_speed = FB_speed + m.moveSpeed
        if event.key == pygame.K_a:
            site[0] += 8
            LR_speed = LR_speed + m.moveSpeed
        if event.key == pygame.K_d:
            site[0] -= 8
            LR_speed = LR_speed - m.moveSpeed
        if event.key == pygame.K_LEFT:
            turn_speed = turn_speed + m.moveSpeed
        if event.key == pygame.K_RIGHT:
            turn_speed = turn_speed - m.moveSpeed
        if event.key == pygame.K_r:
            turn_speed = FB_speed = LR_speed = 0


if __name__ == "__main__":
    # 初始化pygame
    pygame.init()
    # 定义变量
    size = width, height = 350, 400
    bg = (255, 255, 255)
    img0 = pygame.image.load("bg.png")
    # 加载logo图
    img = pygame.image.load("logo.png")
    # 获取图像的位置
    position = img.get_rect().move(70, 90)
    # 创建一个主窗口
    screen = pygame.display.set_mode(size)
    # 标题
    pygame.display.set_caption("麦轮小车")
    # 创建游戏主循环
    m = Motor()
    m.moveSpeed = moveSpeed
    FB_speed = 0
    LR_speed = 0
    turn_speed = 0
    # up = 0
    # down = 180

    # updn_servo = Rotation(23, 25, 170, 170)  # 实例servo舵机引脚23，转动范围25-180度,初始角度170
    # grab_servo = Rotation(24, 90, 185, 90)  # 实例servo舵机引脚24，转动范围90-185度,初始角度90
    # cam_servo = Rotation(25, 100, 150, 90)  # 实例servo舵机引脚24，转动范围90-185度,初始角度90
    # updn_servo.setup()  # 初始化servo舵机
    # grab_servo.setup()  # 初始化servo舵机
    # cam_servo.setup()
    # time.sleep(1)
    flag = 0
    runstate = True
    upmax = 170
    upmin = 35
    upangle =90
    while True:
        # 设置初始值
        site = [0, 0]

        key_list = pygame.key.get_pressed()
        if key_list[pygame.K_UP]:
            servo270(1, upangle)
            runservo()
            if upangle <=upmax:
                upangle = upangle+ 2
        elif key_list[pygame.K_DOWN]:
            servo270(1, upangle)
            runservo()
            if upangle >= upmin:
                upangle = upangle - 5
        for event in pygame.event.get():
            keyboardevent()
            m.move(LR_speed, FB_speed, turn_speed)
            if event.type == pygame.QUIT or runstate == False:
                m.MX_motorUnlockAll()
                sys.exit()
            # time.sleep(0.03)
        # 移动图像
        position = position.move(site)
        # 填充背景
        screen.fill(bg)
        # 放置图片
        screen.blit(img0, (0, 0))
        screen.blit(img, position)
        # 更新显示界面
        pygame.display.flip()
