import threading
import numpy as np
import cv2

class Camera:
    def __init__(self, camera):
        self.frame = []
        self.ret = False
        self.cap = object
        self.camera = camera
        self.openflag = False
    def open(self):
        # if self.cap == object:
        self.cap = cv2.VideoCapture(self.camera)
        self.ret = self.cap.set(3, 320)
        self.ret = self.cap.set(4, 240)
        self.ret = False
        self.openflag = True
        threading.Thread(target=self.queryframe, args=()).start()
    def queryframe(self):
        # self.openflag = True
        # while True:
        while self.openflag:
            self.ret, self.frame = self.cap.read()
            # pass
    def getframe(self):
        return self.ret, self.frame
    def close(self):
        self.openflag = False
        self.cap.release()
def cameratest():
    camera = Camera(0)
    camera.open()
    while True:
        ret,image = camera.getframe()
        if ret:
            print(type(image))
            cv2.imshow('img',image)
            cv2.waitKey(10)
        else:
            print("read faild")
    camera.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # 调试用代码，用来产生控制滑条
    def empty(a):
        pass
    cv2.namedWindow("HSV")
    cv2.resizeWindow("HSV", 640, 300)
    cv2.createTrackbar("HUE Min", "HSV", 95, 179, empty)
    cv2.createTrackbar("SAT Min", "HSV", 110, 255, empty)
    cv2.createTrackbar("VALUE Min", "HSV", 67, 255, empty)
    cv2.createTrackbar("HUE Max", "HSV", 108, 179, empty)
    cv2.createTrackbar("SAT Max", "HSV", 255, 255, empty)
    cv2.createTrackbar("VALUE Max", "HSV", 255, 255, empty)

    targetPos_x = 0
    targetPos_y = 0
    camera = Camera(0)
    camera.open()
    while True:
        ret, img = camera.getframe()
        if ret:
            imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            h_min = cv2.getTrackbarPos("HUE Min", "HSV")
            h_max = cv2.getTrackbarPos("HUE Max", "HSV")
            s_min = cv2.getTrackbarPos("SAT Min", "HSV")
            s_max = cv2.getTrackbarPos("SAT Max", "HSV")
            v_min = cv2.getTrackbarPos("VALUE Min", "HSV")
            v_max = cv2.getTrackbarPos("VALUE Max", "HSV")

            lower = np.array([h_min, s_min, v_min])
            upper = np.array([h_max, s_max, v_max])
            imgMask = cv2.inRange(imgHsv, lower, upper)  # 获取遮罩
            imgOutput = cv2.bitwise_and(img, img, mask=imgMask)
            cv2.imshow('img', img)
            cv2.imshow('imgout', imgOutput)
            cv2.waitKey(10)
        else:
            print("read faild")
    camera.close()
    cv2.destroyAllWindows()


