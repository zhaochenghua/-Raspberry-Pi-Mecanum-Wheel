from TCP_socket import TCP
from motor import Motor

if __name__ == "__main__":
    r = TCP()
    r.MX_Speed = 80  # 预设速度
    m = Motor()
    while True:
        r.recive()
        m.move(r.LR_speed, r.FB_speed, r.turn_speed)
