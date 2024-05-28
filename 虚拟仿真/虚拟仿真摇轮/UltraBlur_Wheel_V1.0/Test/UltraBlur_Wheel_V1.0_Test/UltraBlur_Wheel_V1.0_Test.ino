// 2023.11.6 GHYC
// Test for crane

#define EC1A 14
#define EC1B 12
#define EC2A 32
#define EC2B 33

//#define SW1A 32
//#define SW1B 34

#include "ESP32Encoder.h"
#include "heltec.h"
#include "WiFi.h"=
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include "image.h"


float valWheel1;
float valWheel2;
//float valWheel3;

//bool val_SW1A;
//bool val_SW1B;
//int val_SW1 = 2;
//Wifi
const char* ssid = "DFTT2FPrevis";
// const char* ssid = "DFTT2FStudio";
//const char* ssid = "DFTT_HILS";
// const char* ssid = "DFTT_YanBoTing";
const char* password = "yingshijishuxi";

WiFiUDP Udp;                          // A UDP instance to let us send and receive packets over UDP
IPAddress outIp(255, 255, 255, 255);  // remote IP of your computer
const unsigned int outPort = 9000;    // remote port to receive OSC
const unsigned int localPort = 9000;  // local port to listen for OSC packets (actually not used for sending)

ESP32Encoder encoderWheel1;
ESP32Encoder encoderWheel2;
//ESP32Encoder encoderWheel3;


void setup() {
  Serial.begin(115200);
  //LED灯珠初始化，开机
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  pinMode(EC1A, INPUT);
  pinMode(EC1B, INPUT);
  pinMode(EC2A, INPUT);
  pinMode(EC2B, INPUT);
//  pinMode(EC3A, INPUT);
//  pinMode(EC3B, INPUT);
//  pinMode(SW1A, INPUT_PULLUP);
//  pinMode(SW1B, INPUT_PULLUP);
  pinMode(26, INPUT);
  pinMode(27, INPUT);
  pinMode(34, INPUT);
  pinMode(35, INPUT);
  
  //OLED初始化
  Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Enable*/, true /*Serial Enable*/);
  logo();
  delay(300);

  ESP32Encoder::useInternalWeakPullResistors = UP;  //弱上拉
  encoderWheel1.attachHalfQuad(EC1A, EC1B);
  encoderWheel2.attachHalfQuad(EC2A, EC2B);
//  encoderWheel3.attachHalfQuad(EC3A, EC3B);
  encoderWheel1.setCount(0);
  encoderWheel2.setCount(0);
//  encoderWheel3.setCount(0);

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

  // 三个编码器
  valWheel1 = getEncoderCount(&encoderWheel1, 1000*4);
  valWheel2 = getEncoderCount(&encoderWheel2, 200*4);
//  valWheel3 = getEncoderCount(&encoderWheel3, 30);

//  //开关读取
//  val_SW1A = !digitalRead(SW1A);
//  val_SW1B = !digitalRead(SW1B);
//
//  if (val_SW1A == HIGH && val_SW1B == LOW){
//    int val_SW1 = 1;
//  } else if (val_SW1A == LOW && val_SW1B == LOW){
//    int val_SW1 = 2;
//  } else if (val_SW1A == LOW && val_SW1B == HIGH){
//    int val_SW1 = 3;
//  } 

  OSCMesSend();
  OLEDdisplay();
  while ((micros() % 10000) < 9800) {}
}

float getEncoderCount(ESP32Encoder* encoderWheel,int pulse){
  float valWheel = float(encoderWheel->getCount() % (pulse)) / (pulse) * 360;
  while (valWheel < 0) {
        valWheel += 360;
    }
  return valWheel;
}

void logo() {
  Heltec.display->clear();
  Heltec.display->drawXbm(0, 0, logo_width, logo_height, (const unsigned char*)UltraBlur_Logo);
  Heltec.display->display();
}

void OSCMesSend() {
  //Wheel1
  OSCMessage msg_wheel1("/Wheel/Wheel1");
  msg_wheel1.add(valWheel1);
  Udp.beginPacket(outIp, outPort);
  msg_wheel1.send(Udp);
  Udp.endPacket();
  msg_wheel1.empty();
}

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
  Heltec.display->drawString(25, 0, ssid);
  Heltec.display->drawString(0, 10, outIp.toString() + String(":") + String(localPort));
  //编码器数值显示
  Heltec.display->drawString(0, 20, "Wheel1:");
  Heltec.display->drawString(40, 20, String(valWheel1) + "°");
  Heltec.display->drawString(0, 30, "Wheel2:");
  Heltec.display->drawString(40, 30, String(valWheel2) + "°");
//  Heltec.display->drawString(0, 40, "Wheel3:");
//  Heltec.display->drawString(40, 40, String(valWheel3) + "°");
  //开关状态
//  Heltec.display->drawString(0, 50, "SW1:");
//  Heltec.display->drawString(40, 50, String(val_SW1A));
//  Heltec.display->drawString(50, 50, String(val_SW1B));
//  Heltec.display->drawString(60, 50, String(val_SW1));
  Heltec.display->display();
}

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
