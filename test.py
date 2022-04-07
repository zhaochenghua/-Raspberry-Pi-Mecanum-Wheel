import smbus
import time

bus = smbus.SMBus(1)
pos = [255,90,90,90,90,90,90]

def servo270(port,angle):
    pos[port] = int(angle*180/270)

def runservo():
    for x in pos:
        bus.write_byte(0x08, x)  # 向地址8发送数据
        time.sleep(0.01)

while 1:
    # servo270(120,120,90,90,90,90)
    servo270(1,170)
    servo270(2,170)
    runservo()
    time.sleep(2)  # 延时
    # servo270(90, 90, 90, 90, 90, 90)
    servo270(1,90)
    servo270(2,90)
    runservo()
    time.sleep(2)  # 延时
