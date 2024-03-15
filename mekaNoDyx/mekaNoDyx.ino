#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Servo.h>
int value;

Servo st1;

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
  st1.attach(30);
  st1.write(20);
  pinMode(limA, INPUT);
  pinMode(limB, INPUT);
  pinMode(limS8, INPUT_PULLUP);
  pinMode(limS, INPUT_PULLUP);
  pinMode(limG1, INPUT_PULLUP);
  pinMode(limG2, INPUT_PULLUP);

  pinMode(CWK, OUTPUT);
  pinMode(CCWK, OUTPUT);
  pinMode(PWMK, OUTPUT);

  pinMode(CWT, OUTPUT);
  pinMode(CCWT, OUTPUT);
  pinMode(PWMT, OUTPUT);

  pinMode(cwWorm, OUTPUT);
  pinMode(ccwWorm, OUTPUT);
  pinMode(pwmWorm, OUTPUT);
}

void loop() {
  int stop8 = digitalRead(limS8);
  int stop0 = digitalRead(limS);
  int data = sign;
  Serial.print(stop8);
  Serial.print(stop0);
  Serial.print("  ");
  Serial.print(sign);
  kirim();
  terima();
  switching();
  Serial.println();
  delay(1);
}
void switching() {
  int stop8 = digitalRead(limS8);
  int stop0 = digitalRead(limS);
  int up = digitalRead(limA);
  int down = digitalRead(limB);
  int stopG1 = digitalRead(limG1);
  int stopG2 = digitalRead(limG2);
  int tutup = 100;
  int buka  = 20;
  switch (sign) {
    case L1 :
      if (stop0 == 1) {
        st1.write(buka);
        Serial.print("kiri");
        value = -150;
        motorT(value);
      } else if (stop0 == 0) {
        Serial.print("stopkan");
        st1.write(tutup);
        motorT(0);
      }
      break;
    case R1 :
      if (stop8 == 1) {
        st1.write(buka);
        Serial.print("kanan ");
        value = 150;
        motorT(value);
      } else if (stop8 == 0) {
        Serial.print("stopkir ");
        st1.write(tutup);
        motorT(0);
      }
      break;
    case 169:
      motorWorm(0);
      if ( stop0 == 1 && stop8 == 1) {
        Serial.print("swing");
        motorT(value);
      } else if (stop0 == 0 && stop8 == 1) {
        Serial.print("stop1");
        st1.write(tutup);
        motorT(0);
      } else if (stop8 == 0 && stop0 == 1) {
        Serial.print("stop2");
        st1.write(tutup);
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
      } else if (down == 1) {
        Serial.println("krunturuncir");
        motorK(-50);
      }
      break;

  }

}
