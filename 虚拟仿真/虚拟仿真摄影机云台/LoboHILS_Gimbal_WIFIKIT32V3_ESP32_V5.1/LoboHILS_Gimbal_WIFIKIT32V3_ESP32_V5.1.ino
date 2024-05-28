//最后修改时间 2023年11月22日
//最后修改人：GHYC

#include "ESP32Encoder.h"
#include "heltec.h"
#include "WiFi.h"
#include <WiFiUdp.h>
#include <OSCMessage.h>

#define SW1 47
#define SW2 48
#define JS1X 4
#define JS1Y 5
#define JS1SW 46
#define JS2X 2
#define JS2Y 3
#define JS2SW 45

#include <smooth.h>

#define nbReadings 10
smoother JS1XSmooth(nbReadings);
smoother JS1YSmooth(nbReadings);
smoother JS2XSmooth(nbReadings);
smoother JS2YSmooth(nbReadings);

ESP32Encoder encoderPan;
ESP32Encoder encoderTilt;

float valPan;
float valTilt;
bool val_SW1;
bool val_SW2;
float val_JS1_X;
float val_JS1_Y;
bool val_JS1_SW;
float val_JS2_X;
float val_JS2_Y;
bool val_JS2_SW;
int val_JS1_X_offset = 0;
int val_JS1_Y_offset = 0;
int val_JS2_X_offset = 0;
int val_JS2_Y_offset = 0;

//Wifi
// const char* ssid = "DFTT2FPrevis";
// const char* ssid = "DFTT2FStudio";
const char* ssid = "DFTT_HILS";
// const char* ssid = "DFTT_YanBoTing";
const char* password = "yingshijishuxi";

WiFiUDP Udp;                          // A UDP instance to let us send and receive packets over UDP
IPAddress outIp(255, 255, 255, 255);  // remote IP of your computer
const unsigned int outPort = 9000;    // remote port to receive OSC
const unsigned int localPort = 9000;  // local port to listen for OSC packets (actually not used for sending)

WiFiUDP Udp2;
const unsigned int outPort2 = 8999;
const unsigned int localPort2 = 8999;

//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
void setup() {
  //串口初始化
  Serial.begin(115200);
  //LED灯珠初始化，开机
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  //SW1、SW2、JS1X、JS1Y、JS1SW、JS2X、JS2Y、JS2SW
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  pinMode(JS1X, INPUT);
  pinMode(JS1Y, INPUT);
  pinMode(JS1SW, INPUT);
  pinMode(JS2X, INPUT);
  pinMode(JS2Y, INPUT);
  pinMode(JS2SW, INPUT);

  val_JS1_X_offset = 2047 - analogRead(JS1X);
  val_JS1_Y_offset = 2047 - analogRead(JS1Y);
  val_JS2_X_offset = 2047 - analogRead(JS2X);
  val_JS2_Y_offset = 2047 - analogRead(JS2Y);


  //OLED初始化
  Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Enable*/, true /*Serial Enable*/);
  Heltec.display->screenRotate(ANGLE_270_DEGREE);
  delay(200);
  //编码器初始化
  ESP32Encoder::useInternalWeakPullResistors = UP;  //弱上拉
  encoderPan.attachHalfQuad(41, 34);                //41 42 34
  encoderTilt.attachHalfQuad(39, 33);               //39 40 33
  encoderPan.setCount(5000);                        //起始数值
  encoderTilt.setCount(5000);                       //起始数值

  //WIF初始化
  WIFISetUp();
  Udp.begin(localPort);  //初始化udp


  digitalWrite(LED, LOW);
  //显示logo结束
  Heltec.display->clear();
}

void loop() {
  //指示灯
  digitalWrite(LED, LOW);
  //编码器获取数据
  valPan = float((encoderPan.getCount() + 50000) % 10000) / 5000 - 1;           //编码器5000脉冲每圈，输出数值*2就是10000，归一化之后*360度
  valTilt = float((encoderTilt.getCount() * (-1) + 50000) % 10000) / 5000 - 1;  //编码器5000脉冲每圈，输出数值*2就是10000，归一化之后*360度

  val_SW1 = !digitalRead(SW1);
  val_SW2 = !digitalRead(SW2);
  val_JS1_X = float(joyStackInput(JS1XSmooth.compute(analogRead(JS1X)), val_JS1_X_offset, -1, 20));
  val_JS1_Y = float(joyStackInput(JS1YSmooth.compute(analogRead(JS1Y)), val_JS1_Y_offset, 1, 20));
  val_JS1_SW = !digitalRead(JS1SW);
  val_JS2_X = float(joyStackInput(JS2XSmooth.compute(analogRead(JS2X)), val_JS2_X_offset, -1, 20));
  val_JS2_Y = float(joyStackInput(JS2YSmooth.compute(analogRead(JS2Y)), val_JS2_Y_offset, 1, 20));
  val_JS2_SW = !digitalRead(JS2SW);


  //int average = analogSmooth.compute(analogRead(JS1X));

  OSCMesSend();                         //这是一个自己定义的函数，在下面
  OLEDdisplay();                        //这是一个自己定义的函数，在下面
  while ((micros() % 10000) < 9800) {}  //限制发送频率，等同于delay(20)，但这种方法避免代码执行本身带有delay导致的误差;
}


void OSCMesSend() {
  //pan
  OSCMessage msg_pan("/Tripod1/PanEncoder");
  msg_pan.add(valPan);
  Udp.beginPacket(outIp, outPort);
  msg_pan.send(Udp);
  Udp.endPacket();
  msg_pan.empty();
  //tilt
  OSCMessage msg_tilt("/Tripod1/TiltEncoder");
  msg_tilt.add(valTilt);
  Udp.beginPacket(outIp, outPort);
  msg_tilt.send(Udp);
  Udp.endPacket();
  msg_tilt.empty();
  //SW1 45 \\ SW2 46 \\ JS1X 2 \\ JS1Y 3 \\ JS1SW 4 \\ JS2X 5 \\ JS2Y 6 \\ JS2SW 7
  //JS1_X
  OSCMessage msg_JS1_X("/Tripod1/MovementX");
  msg_JS1_X.add(val_JS1_X);
  Udp.beginPacket(outIp, outPort);
  msg_JS1_X.send(Udp);
  Udp.endPacket();
  msg_JS1_X.empty();
  //JS1_Y
  OSCMessage msg_JS1_Y("/Tripod1/MovementY");
  msg_JS1_Y.add(val_JS1_Y);
  Udp.beginPacket(outIp, outPort);
  msg_JS1_Y.send(Udp);
  Udp.endPacket();
  msg_JS1_Y.empty();
  //JS2_X
  OSCMessage msg_JS2_X("/Tripod1/MovementZ");
  msg_JS2_X.add(val_JS2_Y);
  Udp.beginPacket(outIp, outPort);
  msg_JS2_X.send(Udp);
  Udp.endPacket();
  msg_JS2_X.empty();

  OSCMessage msg_BTN1("/Tripod1/SW1");
  msg_BTN1.add(val_SW1);
  Udp.beginPacket(outIp, outPort);
  msg_BTN1.send(Udp);
  Udp.endPacket();
  msg_BTN1.empty();
  
  OSCMessage msg_BTN2("/Tripod1/SW2");
  msg_BTN1.add(val_SW2);
  Udp.beginPacket(outIp, outPort);
  msg_BTN1.send(Udp);
  Udp.endPacket();
  msg_BTN1.empty();
}



// void OSCBuLSend() {
//   float timetime = millis() % 1000;
//   //pan
//   OSCMessage msg_pan("/Tripod1/PanEncoder");
//   msg_pan.add(valPan);
//   //msg_pan.add(timetime);//---------------debug---------------
//   Udp.beginPacket(outIp, outPort);
//   msg_pan.send(Udp);
//   Udp.endPacket();
//   msg_pan.empty();
//   //tilt
//   OSCMessage msg_tilt("/Tripod1/TiltEncoder");
//   msg_tilt.add(valTilt);
//   Udp.beginPacket(outIp, outPort);
//   msg_tilt.send(Udp);
//   Udp.endPacket();
//   msg_tilt.empty();


// }


void OLEDdisplay() {
  //指示灯
  digitalWrite(LED, LOW);  //灭灯

  //OLED显示
  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  //WIFI状态显示
  Heltec.display->drawString(0, 0, (String)(WiFi.RSSI()) + String("~"));
  //显示wifi的IP地址和端口
  Heltec.display->drawString(20, 0, ssid);
  Heltec.display->drawString(0, 10, outIp.toString() + String(":") + String(localPort));
  //编码器数值显示
  Heltec.display->drawString(0, 20, "Pan:");
  Heltec.display->drawString(30, 20, String(valPan * 180) + "°");
  Heltec.display->drawString(0, 30, " Tilt:");
  Heltec.display->drawString(30, 30, String(valTilt * 180) + "°");
  //摇杆1数值显示
  Heltec.display->drawString(0, 40, "X :");
  Heltec.display->drawString(20, 40, String(val_JS1_X + 0.003));  // 加0.003是防止数值正负跳变
  Heltec.display->drawString(0, 50, "Y :");
  Heltec.display->drawString(20, 50, String(val_JS1_Y + 0.003));
  Heltec.display->drawString(0, 60, "SW1:");
  Heltec.display->drawString(40, 60, bool2String(val_JS1_SW));
  //摇杆2数值显示
  Heltec.display->drawString(0, 80, "X :");
  Heltec.display->drawString(20, 80, String(val_JS2_X + 0.003));
  Heltec.display->drawString(0, 90, "Y :");
  Heltec.display->drawString(20, 90, String(val_JS2_Y + 0.003));
  Heltec.display->drawString(0, 100, "SW2:");
  Heltec.display->drawString(40, 100, bool2String(val_JS2_SW));
  //按钮数值显示
  Heltec.display->drawString(0, 70, "Btn1");
  Heltec.display->drawString(40, 70, bool2String(val_SW1));
  Heltec.display->drawString(0, 110, "Btn2");
  Heltec.display->drawString(40, 110, bool2String(val_SW2));
  //最后才显示
  Heltec.display->display();
}



//--------------------wifi链接--------------------

void WIFISetUp(void) {
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoConnect(true);
  WiFi.begin(ssid, password);
  delay(100);

  Heltec.display->clear();
  byte count = 0;
  while (WiFi.status() != WL_CONNECTED && count < 10) {
    count++;
    delay(500);
    Heltec.display->drawString(0, 0, "Connecting...");
    Heltec.display->display();
  }

  Heltec.display->clear();
  if (WiFi.status() == WL_CONNECTED) {
    Heltec.display->drawString(0, 0, "Connected.");
    Heltec.display->display();
  } else {
    Heltec.display->clear();
    Heltec.display->drawString(0, 0, "Failed to connect.");
    Heltec.display->display();
  }
  Heltec.display->drawString(0, 10, "WiFi Setup done.");
  Heltec.display->display();

  outIp = WiFi.localIP();
  outIp[3] = 255;
  delay(300);
}

String bool2String(bool digitalINPUT) {
  String StrOFF = "OFF";
  String StrON = "ON";
  if (digitalINPUT == 1) {
    return StrON;
  }
  else 
    return StrOFF;
 }



float joyStackInput(int inputVal, int offset, int dir, int deadzone) {  //dir填1或-1
  int temp1 = inputVal + offset - 2048;
  int temp2 = 0;
  if (temp1 > deadzone) {
    temp2 = map(temp1, deadzone, 2047 + offset, 2, 2047);
  } else if (temp1 < -(deadzone)) {
    temp2 = map(temp1, -(deadzone), -2047 + offset, -2, -2047);
  } else {
    temp2 = 0;
  }
  float temp3 = float(temp2) / 2048 * dir;
  return temp3;
}
