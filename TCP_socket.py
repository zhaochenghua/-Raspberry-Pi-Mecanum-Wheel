import socket
import time
import sys

class TCP:
    def __init__(self) -> None:
        self.MX_Speed=10
        self.FB_speed=0
        self.LR_speed=0
        self.turn_speed=0

        self.address = ('192.168.43.182', 7777)#通讯地址
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind(self.address)  # 绑定服务端地址和端口
        self.s.listen(5)
        self.conn, self.addr = self.s.accept()  # 返回客户端地址和一个新的 socket 连接
        print('[+] Connected with', self.addr)
    def recive(self):
        self.data=self.conn.recv(1024)  # buffersize 等于 1024
        self.data=self.data.decode()
        if not self.data: #断开连接没有数据跳出循环结束程序
            sys.exit()
        if(self.data=='FDN'):
            self.FB_speed=self.FB_speed+self.MX_Speed
        elif(self.data=='FUP'):
            self.FB_speed=self.FB_speed-self.MX_Speed
        elif(self.data=='BDN'):
            self.FB_speed=self.FB_speed-self.MX_Speed
        elif(self.data=='BUP'):
            self.FB_speed=self.FB_speed+self.MX_Speed
        elif(self.data=='LDN'):
            self.LR_speed=self.LR_speed-self.MX_Speed
        elif(self.data=='LUP'):
            self.LR_speed=self.LR_speed+self.MX_Speed
        elif(self.data=='RDN'):
            self.LR_speed=self.LR_speed+self.MX_Speed
        elif(self.data=='RUP'):
            self.LR_speed=self.LR_speed-self.MX_Speed
        elif(self.data=='TLDN'):
            self.turn_speed=self.turn_speed-self.MX_Speed
        elif(self.data=='TLUP'):
            self.turn_speed=self.turn_speed+self.MX_Speed
        elif(self.data=='TRDN'):
            self.turn_speed=self.turn_speed+self.MX_Speed
        elif(self.data=='TRUP'):
            self.turn_speed=self.turn_speed-self.MX_Speed
        elif(self.data=='REST'):
            self.turn_speed=self.FB_speed=self.LR_speed=0
        else:
            print('[speed]', self.data)
            try:
                self.MX_Speed=int(self.data)
            except:
                print('error')