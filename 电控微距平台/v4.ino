#define buttonX1 46 
#define buttonX2 45
#define buttonY1 42
#define buttonY2 41
#define buttonZ1 40  
#define buttonZ11 39  
#define buttonZ2 38
#define buttonZ22 26
#define buttonR1 47
#define buttonR2 48

#define stepper1Move 20
#define stepper1Direction 2
#define stepper2Move 3
#define stepper2Direction 4
#define stepper3Move 5
#define stepper3Direction 6
#define stepper4Move 7
#define stepper4Direction 19

#define ShootHalf 36
#define ShootAll 37

#include <ESP_FlexyStepper.h>
// 创建四个步进电机
ESP_FlexyStepper stepper1;
ESP_FlexyStepper stepper2;
ESP_FlexyStepper stepper3;
ESP_FlexyStepper stepper4;

const int stackPulse = 130;//z轴每一步移动多少距离（特指仅z平面堆栈时）
const int stackPhotoNumber = 10;//堆栈的照片数量(特指仅z平面堆栈时)
const int stackStopDelay = 500;     //移动完之后等待的时间 单位是ms，1s=1000ms
const int stackShutterDelay = 500;  //触发快门后等待的时间

const int stackPulseX = 200;//x轴每一步移动多少距离
const int stackPulseY = 200;//y轴每一步移动多少距离
const int stackPulseR = 200;//r轴每一步移动多少距离
const int stackPulseZ = 200;//r轴每一步移动多少距离

const int stackPulseXfu = -200;//x轴每一步移动多少距离
const int stackPulseYfu = -200;//y轴每一步移动多少距离
const int stackPulseRfu = -200;//r轴每一步移动多少距离
const int stackPulseZfu = -200;//r轴每一步移动多少距离

const int stackPhotoNumberX = 30;//x轴栈的照片数量
const int stackPhotoNumberY = 1;//y轴堆栈的照片数量
const int stackPhotoNumberZ = 5;//z轴堆栈的照片数量
const int stackPhotoNumberR = 1;//r轴堆栈的照片数量

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  //读取四个按钮、两个摇杆引脚
  pinMode(buttonX1, INPUT);   //按键
  pinMode(buttonX2, INPUT);   //按键
  pinMode(buttonY1, INPUT);  //按键
  pinMode(buttonY2, INPUT);  //按键
  pinMode(buttonZ1, INPUT);   //按键
  pinMode(buttonZ11, INPUT);   //按键
  pinMode(buttonZ2, INPUT);   //按键
  pinMode(buttonZ22, INPUT);   //按键
  pinMode(buttonR1, INPUT);  //按键
  pinMode(buttonR2, INPUT);  //按键
  
  stepper1.connectToPins(stepper1Move, stepper1Direction);
  stepper2.connectToPins(stepper2Move, stepper2Direction);
  stepper3.connectToPins(stepper3Move, stepper3Direction);
  stepper4.connectToPins(stepper4Move, stepper4Direction);

  stepper1.setSpeedInStepsPerSecond(10000);  // 调整速度
  stepper1.setAccelerationInStepsPerSecondPerSecond(10000);  // 调整加速度
  stepper2.setSpeedInStepsPerSecond(10000);
  stepper2.setAccelerationInStepsPerSecondPerSecond(10000);
  stepper3.setSpeedInStepsPerSecond(500000);
  stepper3.setAccelerationInStepsPerSecondPerSecond(500000);
  stepper4.setSpeedInStepsPerSecond(10000);
  stepper4.setAccelerationInStepsPerSecondPerSecond(10000);

  pinMode(ShootHalf, OUTPUT);     //半按快门
  pinMode(ShootAll, OUTPUT);     //全按快门
  digitalWrite(ShootHalf, HIGH);  //高电平为普通状态，低电平才是触发快门
  digitalWrite(ShootAll, HIGH);  //高电平为普通状态，低电平才是触发快门
}

void loop() {
  while (digitalRead(buttonX1) == HIGH) {
    stepper1.setTargetPositionRelativeInSteps(10);
    stepper1.processMovement();
  }
  
  while (digitalRead(buttonX2) == HIGH) {
    stepper1.setTargetPositionRelativeInSteps(-10);
    stepper1.processMovement();
  }
  
  while (digitalRead(buttonY1) == HIGH) {
    stepper2.setTargetPositionRelativeInSteps(10);
    stepper2.processMovement();
  }
  
  while (digitalRead(buttonY2) == HIGH) {
    stepper2.setTargetPositionRelativeInSteps(-10);
    stepper2.processMovement();
  }
  while (digitalRead(buttonZ1) == LOW) {
    stepper3.setTargetPositionRelativeInSteps(10);
    stepper3.processMovement();
  }
  
  while (digitalRead(buttonZ11) == LOW) {
    stepper3.setTargetPositionRelativeInSteps(50);
    stepper3.processMovement();
  }
  
  while (digitalRead(buttonZ2) == LOW) {
    stepper3.setTargetPositionRelativeInSteps(-10);
    stepper3.processMovement();
  }
  
  while (digitalRead(buttonZ22) == LOW) {
    stepper3.setTargetPositionRelativeInSteps(-50);
    stepper3.processMovement();
  }
  while (digitalRead(buttonR1) == HIGH) {
    stepper4.setTargetPositionRelativeInSteps(10);
    stepper4.processMovement();
  }
  
  while (digitalRead(buttonR2) == HIGH) {
    stepper4.setTargetPositionRelativeInSteps(-10);
    stepper4.processMovement();
  }

  if (Serial.available() > 0) {
    char shutter = Serial.read();
    if (shutter == 'S' || shutter == 's') {  
     shutterRel();
    }
    else if (shutter == 'E' || shutter == 'e') {
     shutterStack(stackPulse, stackPhotoNumber, stackStopDelay, stackShutterDelay);
    }
    else if (shutter == 'x') {
     shutterStackX(stackPulseX, stackPhotoNumberX, stackStopDelay, stackShutterDelay);
    }
    else if (shutter == 'y') {
     shutterStackY(stackPulseY, stackPhotoNumberY, stackStopDelay, stackShutterDelay);
    }
    else if (shutter == 'z') {
     shutterStackZ(stackPulseZ, stackPhotoNumberZ, stackStopDelay, stackShutterDelay);
    }
    else if (shutter == 'r') {
     shutterStackR(stackPulseR, stackPhotoNumberR, stackStopDelay, stackShutterDelay);
    }
    else if (shutter == 'X') {
     shutterStackX(stackPulseXfu, stackPhotoNumberX, stackStopDelay, stackShutterDelay);
    }
    else if (shutter == 'Y') {
     shutterStackY(stackPulseYfu, stackPhotoNumberY, stackStopDelay, stackShutterDelay);
    }
    else if (shutter == 'Z') {
     shutterStackZ(stackPulseZfu, stackPhotoNumberZ, stackStopDelay, stackShutterDelay);
    }
    else if (shutter == 'R') {
     shutterStackR(stackPulseRfu, stackPhotoNumberR, stackStopDelay, stackShutterDelay);
    }
    else if (shutter == 'H' || shutter == 'h') {
     shutterStackpro(stackPulseX, stackPulseY, stackPulseZ, stackPulseR, stackPhotoNumberX, stackPhotoNumberY, stackPhotoNumberZ, stackPhotoNumberR, stackStopDelay, stackShutterDelay);
    }
  }
}

void shutterRel() {
  digitalWrite(ShootHalf, LOW);   //触发半按快门
  delay(500);             //等待100毫秒
  digitalWrite(ShootAll, LOW);   //觧发全按快门
  delay(500);             //等待200毫秒
  digitalWrite(ShootAll, HIGH);  //松开全按快门
  digitalWrite(ShootHalf, HIGH);  //松开半按快门
}

void shutterStack(int pulse, int photoNumber, int stopDelay, int shutterDelay) {
  for (int j = 0; j < photoNumber; j++) {
    delay(stopDelay);
    shutterRel();
    delay(shutterDelay);
    stepper3.moveRelativeInSteps(pulse);
  }
}
void shutterStackX(int pulse, int photoNumber, int stopDelay, int shutterDelay) {
  for (int j = 0; j < photoNumber; j++) {
    delay(stopDelay);
    shutterRel();
    delay(shutterDelay);
    stepper1.moveRelativeInSteps(pulse);    
  }
}
void shutterStackY(int pulse, int photoNumber, int stopDelay, int shutterDelay) {
  for (int j = 0; j < photoNumber; j++) {
    delay(stopDelay);
    shutterRel();
    delay(shutterDelay);
    stepper2.moveRelativeInSteps(pulse);    
  }
}
void shutterStackZ(int pulse, int photoNumber, int stopDelay, int shutterDelay) {
  for (int j = 0; j < photoNumber; j++) {
    delay(stopDelay);
    shutterRel();
    delay(shutterDelay);
    stepper3.moveRelativeInSteps(pulse);
  }
}
void shutterStackR(int pulse, int photoNumber, int stopDelay, int shutterDelay) {
  for (int j = 0; j < photoNumber; j++) {
    delay(stopDelay);
    shutterRel();
    delay(shutterDelay);
    stepper4.moveRelativeInSteps(pulse);    
  }
}
void shutterStackXfu(int pulse, int photoNumber, int stopDelay, int shutterDelay) {
  for (int j = 0; j < photoNumber; j++) {
    delay(stopDelay);
    shutterRel();
    delay(shutterDelay);
    stepper1.moveRelativeInSteps(pulse);
  }
}
void shutterStackYfu(int pulse, int photoNumber, int stopDelay, int shutterDelay) {
  for (int j = 0; j < photoNumber; j++) {
    delay(stopDelay);
    shutterRel();
    delay(shutterDelay);
    stepper2.moveRelativeInSteps(pulse);   
  }
}
void shutterStackZfu(int pulse, int photoNumber, int stopDelay, int shutterDelay) {
  for (int j = 0; j < photoNumber; j++) {
    delay(stopDelay);
    shutterRel();
    delay(shutterDelay);
    stepper3.moveRelativeInSteps(pulse);    
  }
}
void shutterStackRfu(int pulse, int photoNumber, int stopDelay, int shutterDelay) {
  for (int j = 0; j < photoNumber; j++) {
    delay(stopDelay);
    shutterRel();
    delay(shutterDelay);
    stepper4.moveRelativeInSteps(pulse);    
  }
}


void shutterStackpro(int pulsex, int pulsey, int pulsez, int pulser, int photoNumberx, int photoNumbery, int photoNumberz, int photoNumberr, int stopDelay, int shutterDelay) {
  for (int j = 0; j <= photoNumberx; j++) {
    stepper1.moveRelativeInSteps(pulsex);
    for (int k = 0; k <= photoNumbery; k++) {
      stepper2.moveRelativeInSteps(pulsey);
      for (int l = 0; l <= photoNumberr; l++) {
        stepper3.moveRelativeInSteps(pulser);
        for (int m = 0; m <= photoNumberz; m++) {
          stepper4.moveRelativeInSteps(pulsez);
          delay(1000);  
          delay(stopDelay);
          shutterRel();
          delay(shutterDelay);
          
        }
      }
    }
  }
}

