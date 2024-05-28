// UltraBlur
// 虚拟仿真伸缩炮 V1.2
// 2023-11-24

#include "ESP32Encoder.h"
#include "heltec.h"
#include "WiFi.h"
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include "image.h"
#include <smooth.h>
#include <Bounce2.h>

#define EC1A 14
#define EC1B 12
#define EC2A 32
#define EC2B 33

#define JS1X 37
#define JS1Y 38
#define JS1SW 22

float valWheel1;
float valWheel1_normalize;
float valWheel2_normalize;

#define nbReadings 10
smoother JS1XSmooth(nbReadings);
smoother JS1YSmooth(nbReadings);

ESP32Encoder encoderWheel1;
ESP32Encoder encoderWheel2;

Bounce debouncer;

float val_JS1_X;
float val_JS1_Y;
bool val_JS1_SW;
bool jS_Mode;
int val_JS1_X_offset = 0;
int val_JS1_Y_offset = 0;

unsigned long max_tilt = 2147483648;;
unsigned long min_tilt = 2147483647;;

//Wifi
//const char* ssid = "DFTT2FPrevis";
// const char* ssid = "DFTT2FStudio";
const char* ssid = "DFTT_HILS";
// const char* ssid = "DFTT_YanBoTing";
const char* password = "yingshijishuxi";

WiFiUDP Udp;                                // A UDP instance to let us send and receive packets over UDP
const IPAddress outIp(255, 255, 255, 255);  // remote IP of your computer
const unsigned int outPort = 9001;          // remote port to receive OSC
const unsigned int localPort = 9001;        // local port to listen for OSC packets (actually not used for sending)


void setup() {
  //analogSetAttenuation(ADC_11db); 
  Serial.begin(115200);
  //LED灯珠初始化，开机
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  pinMode(EC1A, INPUT);
  pinMode(EC1B, INPUT);
  pinMode(EC2A, INPUT);
  pinMode(EC2B, INPUT);
  pinMode(JS1X, INPUT);
  pinMode(JS1Y, INPUT);
  pinMode(JS1SW, INPUT_PULLUP);
  
  debouncer.attach(JS1SW); // 将Bounce对象与引脚绑定
  debouncer.interval(10); // 设置消抖间隔（以毫秒为单位）
  
  val_JS1_X_offset = 2047 - analogRead(JS1X);
  val_JS1_Y_offset = 2047 - analogRead(JS1Y);
  //默认为True，为伸缩炮模式
  jS_Mode = true; 
  
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
  encoderWheel1.setCount(0);
  encoderWheel2.setCount(0);

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

  // 二个编码器
  valWheel1 = getEncoderCount_360(&encoderWheel1, 4000*4);
  valWheel1_normalize = valWheel1/360;
  valWheel2_normalize = getEncoderCount_maxmin(&encoderWheel2, 5000*4);

  // 摇杆
  val_JS1_X = float(joyStackInput(JS1XSmooth.compute(analogRead(JS1X)), val_JS1_X_offset, -1, 400));
  val_JS1_Y = float(joyStackInput(JS1YSmooth.compute(analogRead(JS1Y)), val_JS1_Y_offset, 1, 400));
  Serial.print(analogRead(JS1X));
  Serial.print('\n');
  debouncer.update();
  val_JS1_SW = debouncer.rose();

  
  if (val_JS1_SW) {
    jS_Mode = !jS_Mode;
  }

  
  OSCMesSend();
  OLEDdisplay();
  while ((micros() % 10000) < 9800) {}
}

float getEncoderCount_360(ESP32Encoder* encoderWheel,int pulse){
  float valWheel = float(encoderWheel->getCount() % (pulse)) / (pulse) * 360;
  while (valWheel < 0) {
        valWheel += 360;
    }
  return valWheel;
}

float getEncoderCount_maxmin(ESP32Encoder* encoderWheel,int pulse){
  unsigned long val_tilt;
  float val_tilt_float;
  val_tilt = encoderWheel->getCount() + 2147483648;
  if (val_tilt > max_tilt)max_tilt = val_tilt;
  if (val_tilt < min_tilt)min_tilt = val_tilt;
  val_tilt_float = float(val_tilt - min_tilt) / float(max_tilt - min_tilt);
  return val_tilt_float;
}

void logo() {
  Heltec.display->clear();
  Heltec.display->drawXbm(0, 0, logo_width, logo_height, (const unsigned char*)UltraBlur_Logo);
  Heltec.display->display();
}

void OSCMesSend() {
  OSCMessage msg_wheel1("/Telescopic/Pan");
  msg_wheel1.add(valWheel1_normalize);
  Udp.beginPacket(outIp, outPort);
  msg_wheel1.send(Udp);
  Udp.endPacket();
  msg_wheel1.empty();

  OSCMessage msg_wheel2("/Telescopic/Tilt");
  msg_wheel2.add(valWheel2_normalize);
  Udp.beginPacket(outIp, outPort);
  msg_wheel2.send(Udp);
  Udp.endPacket();
  msg_wheel2.empty();

  if (jS_Mode){
    OSCMessage msg_extend("/Telescopic/Extend");
    msg_extend.add(val_JS1_Y);
    Udp.beginPacket(outIp, outPort);
    msg_extend.send(Udp);
    Udp.endPacket();
    msg_extend.empty();
  }else{
    OSCMessage msg_x("/Telescopic/moveX");
    msg_x.add(val_JS1_X);
    Udp.beginPacket(outIp, outPort);
    msg_x.send(Udp);
    Udp.endPacket();
    msg_x.empty();

    OSCMessage msg_y("/Telescopic/moveY");
    msg_y.add(val_JS1_Y);
    Udp.beginPacket(outIp, outPort);
    msg_y.send(Udp);
    Udp.endPacket();
    msg_y.empty();
  }
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
  Heltec.display->drawString(0, 10, WiFi.localIP().toString() + String(":") + String(localPort));
  //编码器数值显示
  Heltec.display->drawString(0, 20, "Pan:");
  Heltec.display->drawString(40, 20, String(valWheel1) + "°");
  Heltec.display->drawString(0, 30, "Tilt:");
  Heltec.display->drawString(40, 30, String(valWheel2_normalize));
  Heltec.display->drawString(0, 40, "X:");
  Heltec.display->drawString(15, 40, String(val_JS1_X + 0.003));
  Heltec.display->drawString(40, 40, "Y:");
  Heltec.display->drawString(55, 40, String(val_JS1_Y + 0.003));
  Heltec.display->drawString(0, 50, "SW:");
  Heltec.display->drawString(20, 50, bool2String(val_JS1_SW));
  Heltec.display->drawString(40, 50, "Mode:" );
  Heltec.display->drawString(70, 50, bool2String(jS_Mode));
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

  IPAddress outIp;
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
