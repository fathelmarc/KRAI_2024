#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Servo.h>
const int gripper = 45;
const int lifter = 43;
const int lifter1 = 34;
const int gripper1 = 36;
const int ballLifter = 13;
const int extender = 0;

const int lamp1 = 31;
const int lamp2 = 33;
const int lamp3 = 39;
const int lamp4 = 49;

const int CW_left = 0;
const int CCW_left = 0;
const int PWM_left = 0;

const int CW_right= 0;
const int CCW_right= 0;
const int PWM_right= 0;

const int CW_launch= 0;
const int CCW_launch= 0;
const int PWM_launch= 0;

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 0, 70);//ip address arduino
IPAddress gateway(192, 168, 0, 6);//gateway
IPAddress subnet(255, 255, 255, 0); //netmask
IPAddress myDns(8, 8, 8, 8); //tak penting mungkin
IPAddress serverIP(192, 168, 0, 7);//ip address laptop
unsigned int localPort = 8888;
char packetBuffer[512];
EthernetUDP Udp;

struct gy25 {
  unsigned char buffer[8];
  unsigned char counter;
  float yaw;
  float roll;
  float pitch;
} imu;
int gyro = 0;
int imu_pure = 0;

#define SILANG 0
#define CIRCLE 1
#define TRIANGLE 2
#define SQUARE 3
#define L1 4
#define R1 5
#define L2 6
#define R2 7
const int CIRCLE1 = -1;
const int SQUARE1 = -3;

#define shortenSign 79
#define extendSign 97
#define launcherOn 756
#define launcherOff 765

#define tutup 1
#define buka 0

#define angkat 1
#define turun 0

#define extend 0
#define shorten 1
int i, j, lamp, value, sign, tanda;

const int resetPin_nano = 3;

void setup() {
  Ethernet.init(53);
  Ethernet.begin(mac, ip, myDns, gateway, subnet);
  Udp.begin(localPort);
  Serial.begin(115200);

  pinMode(CW_left, OUTPUT);
  pinMode(CCW_left, OUTPUT);
  pinMode(PWM_left, OUTPUT);
  
  pinMode(CW_right, OUTPUT);
  pinMode(CCW_right, OUTPUT);
  pinMode(PWM_right, OUTPUT);
  
  pinMode(CW_launch, OUTPUT);
  pinMode(CCW_launch, OUTPUT);
  pinMode(PWM_launch, OUTPUT);

  pinMode(lifter, OUTPUT);// 1 bawah, 0 atas
  pinMode(gripper, OUTPUT);// 0 grip, 1 buka
  pinMode(lifter1, OUTPUT);// 1 bawah, 0 atas
  pinMode(gripper1, OUTPUT);// 0 grip, 1 buka
  pinMode(ballLifter, OUTPUT);
  pinMode(extender,  OUTPUT);

  pinMode(resetPin_nano, OUTPUT);

  pinMode(lamp1, OUTPUT);
  pinMode(lamp2, OUTPUT);
  pinMode(lamp3, OUTPUT);
  pinMode(lamp4, OUTPUT);

  digitalWrite(lamp1, LOW);
  digitalWrite(lamp2, LOW);
  digitalWrite(lamp3, LOW);
  digitalWrite(lamp4, LOW);

}

void(* resetFunc) (void) = 0;
void loop() {
  kirim();
  terima();
  changer();

  switch (sign) {
    case launcherOn:
      launcher(200,-200);
      break;
    case launcherOff:
      launcher(0,0);
      break;
    case extendSign:
      digitalWrite(extender, extend);
      break;
    case shortenSign:
      digitalWrite(extender, shorten);
      break;
    case 78:
      digitalWrite(ballLifter, HIGH);
      break;
    case 87:
      digitalWrite(ballLifter, LOW);
      break;
    case 999:
      digitalWrite(resetPin_nano, HIGH);
      delay(100);
      resetFunc();
      break;
    case 90:
      value = -150;
      digitalWrite(gripper1, buka);
      digitalWrite(gripper, buka);
      digitalWrite(extender, extend);
      break;
    case SQUARE:
      digitalWrite(gripper, tutup);
      digitalWrite(lifter, angkat);
      i = 0;
      break;
    case CIRCLE:
      digitalWrite(lifter, turun);
      while (i < 1501) {
        Serial.println(i);
        i++;
        if (i == 1500 || sign != CIRCLE) {
          break;
        }
      }
      digitalWrite(gripper, buka);
      break;
    case SQUARE1:
      digitalWrite(gripper1, tutup);
      digitalWrite(lifter1, angkat);
      j = 0;
      break;
    case CIRCLE1:
      digitalWrite(lifter1, turun);
      while (j < 1501) {
        Serial.println(i);
        j++;
        if (j == 1500 || sign != CIRCLE1) {
          break;
        }
      }
      digitalWrite(gripper1, buka);
      break;
  }

}
