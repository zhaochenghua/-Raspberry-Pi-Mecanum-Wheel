import threading
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
# def cameratest():
#     camera = Camera(0)
#     camera.open()
#     while True:
#         ret,image = camera.getframe()
#         if ret:
#             print(type(image))
#             cv2.imshow('img',image)
#             cv2.waitKey(10)
#         else:
#             print("read faild")
#     camera.close()
#     cv2.destroyAllWindows()
