// I2c 舵机控制板，使用arduino nano主控，I2c从机模式,地址为#8
// 舵机连接口为3，5，6，9，10，11
// 接收7字节数据，第一位0xFF为标志位，其余对应上述舵机口想要达到的角度。
#include <Wire.h>
#include <Servo.h>

Servo myservo[6];  // create servo object to control a servo
int pos[6]; //= {90, 90, 90, 90, 90, 90};
int targetPos[6]; //= {60, 70, 80, 90, 100, 110};
int flag = 0;
int num = 0;
int state = 0;
void setup() {

  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  //  Serial.begin(9600);
}

void loop() {

  while (pos[0] != targetPos[0] || pos[1] != targetPos[1] || pos[2] != targetPos[2] || pos[3] != targetPos[3] || pos[4] != targetPos[4] || pos[5] != targetPos[5])
  {
    for (int i = 0; i < 6; i++) {
      mv(i, targetPos[i]);
      //Serial.print(targetPos[i]);
    }
  }
}

void mv(int o, int p) {//o是端口号，p是需要到达的角度
  if (p > pos[o])  {
    myservo[o].write(pos[o] + 1);
    pos[o] += 1;
    delay(1);
  }
  if (p < pos[o]) {
    myservo[o].write(pos[o] - 1);
    pos[o] -= 1;
    delay(1);
  }
}

void receiveEvent(int howMany) {
  int x = Wire.read();    // receive byte as an integer
  //  Serial.print(x);
  if (flag == 1 && x <= 180) {//如果收到过头了，且收到的数据小于180
    targetPos[num] = x;
    num ++;
    if (num == 6 ) {
      flag = 0;
      if (state == 0) {//如果未初始化
        myservo[0].attach(3);  // attaches the servo on pin 3 to the servo object
        myservo[0].write(targetPos[0]);
        myservo[1].attach(5);
        myservo[1].write(targetPos[1]);
        myservo[2].attach(6);
        myservo[2].write(targetPos[2]);
        myservo[3].attach(9);
        myservo[3].write(targetPos[3]);
        myservo[4].attach(10);
        myservo[4].write(targetPos[4]);
        myservo[5].attach(11);
        myservo[5].write(targetPos[5]);
        state = 1;
      }
    }
  }
  else if (x == 255) {
    flag = 1;
    num = 0;
  }
}
