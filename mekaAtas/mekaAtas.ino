#include <Dynamixel2Arduino.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
const int ctrl = 6;
Dynamixel2Arduino dxl(Serial2, ctrl);
int value;

#define SILANG 0
#define CIRCLE 1
#define TRIANGLE 2
#define SQUARE 3
#define L1 4
#define R1 5
#define L2 6
#define R2 7
#define gripWorm 79
#define releaseWorm 70

int sign;
String chat;
const uint8_t id1 = 1;
const uint8_t id2 = 2;
const uint8_t id3 = 3;
const uint8_t id4 = 4;
using namespace ControlTableItem;
#define buka1 2601
#define tutup1 1620
#define buka2 2600
#define tutup2 3470
//#define tutup
const int sole = 11;

const int CWK = 27;
const int CCWK = 29;
const int PWMK = 4;

const int cwWorm = 7;
const int ccwWorm = 9;
const int pwmWorm = 8;

const int CWT = 25;
const int CCWT = 23;
const int PWMT = 3;

const int limA = 33;
const int limB = 31;

const int limS8 = 37;
const int limS = 35;

const int limG1 = 39;
const int limG2 = 41;

const int limA2 = 43;
const int limB2 = 45;

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

IPAddress ip(192, 168, 0, 70);//ip address arduino
IPAddress gateway(192, 168, 0, 1);//gateway
IPAddress subnet(255, 255, 255, 0); //netmask
IPAddress myDns(8, 8, 8, 8); //tak penting mungkin

IPAddress serverIP(192, 168, 0, 7);//ip address laptop

unsigned int localPort = 5555;
char packetBuffer[512];

EthernetUDP Udp;

struct gy25 {
  char buffer[50];
  int counter;
  float heading;
} cmps;

int prevWaktu;

void setup() {
  // put your setup code here, to run once:
  Ethernet.init(53);
  Ethernet.begin(mac, ip, myDns, gateway, subnet);
  Udp.begin(localPort);

  Serial.begin(115200);
  kalibrasi();
  dxl.begin(115200);
  dxl.setPortProtocolVersion(2.0);
  dxl.ping(id1);
  dxl.ping(id2);//available
  dxl.ping(id3);//available
  dxl.ping(id4);

  dxl.torqueOff(id1);
  dxl.torqueOff(id2);
  dxl.torqueOff(id3);
  dxl.torqueOff(id4);

  dxl.setOperatingMode(id1, OP_POSITION);
  dxl.setOperatingMode(id2, OP_POSITION);
  dxl.setOperatingMode(id3, OP_POSITION);
  dxl.setOperatingMode(id4, OP_POSITION);

  dxl.torqueOn(id1);
  dxl.torqueOn(id2);
  dxl.torqueOn(id3);
  dxl.torqueOn(id4);

  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  dxl.writeControlTableItem(PROFILE_VELOCITY, id1, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, id2, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, id3, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, id4, 0);

  pinMode(sole, OUTPUT);
  digitalWrite(sole,1);
  pinMode(limA, INPUT);
  pinMode(limB, INPUT);
  pinMode(limS8, INPUT_PULLUP);
  pinMode(limS, INPUT_PULLUP);

  pinMode(CWK, OUTPUT);
  pinMode(CCWK, OUTPUT);
  pinMode(PWMK, OUTPUT);

  pinMode(CWT, OUTPUT);
  pinMode(CCWT, OUTPUT);
  pinMode(PWMT, OUTPUT);

  pinMode(cwWorm, OUTPUT);
  pinMode(ccwWorm, OUTPUT);
  pinMode(pwmWorm, OUTPUT);

  pinMode(limG1, INPUT);
  pinMode(limG2, INPUT);
}

void loop() {
  int stop8 = digitalRead(limS8);
  int stop0 = digitalRead(limS);
  int data = sign;
  Serial.print(sign);
//  Serial.print(stop8);
//  Serial.print(stop0);
  kirim();
  terima();
  switching();
  Serial.println();
//  delay(1);/
}
void switching() {
  int stop8 = digitalRead(limS8);
  int stop0 = digitalRead(limS);
  int up = digitalRead(limA);
  int down = digitalRead(limB);
  int stopG1 = digitalRead(limG1);
  int stopG2 = digitalRead(limG2);
  switch (sign) {
    case L1 :
      if (stop0 == 1) {
        Serial.print("kiri");
//        digitalWrite(sole,0);/
        delay(100);
        value = -150;
        motorT(value);
      } else if (stop0 == 0) {
        Serial.print("stopkan");
//        digitalWrite(sole,1);/
        delay(100);
        motorT(0);
      }
      break;
    case R1 :
      if (stop8 == 1) {
        Serial.print("kanan ");
//        digitalWrite(sole,0);/
        delay(100);
        value = 150;
        motorT(value);
      } else if (stop8 == 0) {
        Serial.print("stopkir ");
//        digitalWrite(sole,1);/
        delay(100);
        motorT(0);
      }
      break;
    case 169:
      motorWorm(0);
//        digitalWrite(sole,1);/
      if ( stop0 == 1 && stop8 == 1) {
        Serial.print("swing");
        motorT(value);
      } else if (stop0 == 0 && stop8 == 1) {
        Serial.print("stop1");
        motorT(0);
      } else if (stop8 == 1 && stop0 == 0) {
        Serial.print("stop2");
        motorT(0);
      }
      break;
    case SQUARE:
      if (up == 0) {
        Serial.print("krunstop squ  ");
        motorK(0);
      } else {
        if (stopG1 == 0) {
          motorWorm(100);
          Serial.println("stopgriping ");
        } else {
          Serial.println("grip");
          motorWorm(-255);
        }
        Serial.println("krunnaiksqu");
        motorK(50);
      }
      break;
    case CIRCLE:
      if (down == 0) {
        Serial.print("krunstop cir");
        motorK(0);
        motorWorm(255);
        //        if (millis() < 1000) {
        //          Serial.println("owrm mati");
        //          motorWorm(0);
        //        }
      } else if (down == 1) {
        Serial.println("krunturuncir");
        motorK(-50);
      }
      break;

  }

}
