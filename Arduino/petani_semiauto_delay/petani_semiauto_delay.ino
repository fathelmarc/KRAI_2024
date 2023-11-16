#include <PS3BT.h>
#include <usbhub.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(36, 34);  //RX,TX
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
BTD Btd(&Usb);
PS3BT PS3(&Btd);

#define encA3 2  //motor 3
#define encB3 46
#define encA2 3  //motor 2
#define encB2 40
#define encA1 18  //motor 1
#define encB1 42
#define encA4 19  //motor 4
#define encB4 48

#define CW 23
#define CCW 25
#define PWM 5

#define relay 11

//bluetooth

#define R_roda 0.05
#define R_rbt 22.184  //0.22314
#define R_robot 0.16149
#define totdeg 360
#define PPR 200
#define conversi_g 19.2
float gp = PPR * conversi_g;
float k_roda = 2 * M_PI * R_roda;
volatile float pos2, pos1, pos3, pos4,
  posprev1, posprev2, posprev3, posprev4;
float x, y, t, posisi_x, posisi_y, posisi_t,
  hadapFilt, hadapPrev, hadap,
  Xprev, Yprev, Tprev,
  v1Prev, v1Filt, v2Prev, v2Filt,
  v3Prev, v3Filt, v4Prev, v4Filt,
  deltaT,
  errTprev, errYprev, errXprev;
bool mode;
long prevT;
int speedMax = 1, rotateMax = 1;
struct gy25 {
  char buffer[50];
  int counter;
  float heading;
} cmps;

float torad(float sudut) {
  return sudut * PI / 180;
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial3.begin(115200);
  mySerial.begin(38400);
  calibrate_IMU();
#if !defined(__MIPSEL__)
  while (!Serial)
    ;
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1)
      ;
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
  pinMode(CW, OUTPUT);
  pinMode(CCW, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(relay, OUTPUT);
  pinMode(38, INPUT_PULLUP);
}
void loop() {
  Usb.Task();
  float imunow = hadap;
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
    if (PS3.getAnalogHat(LeftHatX) > 137 || PS3.getAnalogHat(LeftHatX) < 117 || PS3.getAnalogHat(LeftHatY) > 137 || PS3.getAnalogHat(LeftHatY) < 117 || PS3.getAnalogHat(RightHatX) > 137 || PS3.getAnalogHat(RightHatX) < 117 || PS3.getAnalogHat(RightHatY) > 137 || PS3.getAnalogHat(RightHatY) < 117) {
      float hx = (PS3.getAnalogHat(LeftHatX) - 127.5) * speedMax / 127.5;
      float hy = (-PS3.getAnalogHat(LeftHatY) + 127.5) * speedMax / 127.5;
      float ht = (PS3.getAnalogButton(L2) - PS3.getAnalogButton(R2)) * rotateMax / 255.0;
      if (mode == true) {
        hx = -hx;
        hy = -hy;
      }
      koordinat(hx, hy, imunow);
    } else if (PS3.getButtonClick(UP)) {

      koordinat(0, 1, 0);
      delay(357.14285714285714285714285714286);
      koordinat(0.5, 0, 0);
      for (int x = 0; x < 2142.85; x++)
        delay(1);
      koordinat(0.5, 0, 0);
      for (int x = 0; x < 3928.57; x++) {
        delay(1);
      }
    } else if (PS3.getAnalogButton(CIRCLE)) {
      koordinat(0, 0, 45);
      Serial.println(90);
    } else if (PS3.getAnalogButton(TRIANGLE)) {
      koordinat(0, 0, 0);
      Serial.println(0);
    } else if (PS3.getAnalogButton(SQUARE)) {
      koordinat(0, 0, -90);
      Serial.println(-90);
    } else if (PS3.getAnalogButton(R2)) {
      setMotor(1, 40, PWM, CW, CCW);
    } else if (PS3.getAnalogButton(L2)) {
      setMotor(2, 255, PWM, CW, CCW);
    } else if (PS3.getButtonClick(PS)) {
      setMotor(0, 0, PWM, CW, CCW);
    } else {
      setMotor(0, 0, PWM, CW, CCW);
      koordinat(0, 0, imunow);
    }
  }
}

void koordinat(float tx, float ty, float tHadap) {
  float kp = 0.03;  //0.36;//0.6;
  float ki = 0;     //55.5371;//43.46;
  float kd = 0;     //13.8843;//10.86;
  updateCMPS();
  //waktu
  long currT = micros();
  deltaT = (float)(currT - prevT) / 1.0e6;
  //PID theta
  hadap = cmps.heading * -1;
  float errT = tHadap - hadap;
  if (errT > 180 || errT < -180) {
    errT = hadap - tHadap;
  }
  float edervT = (errT - errTprev) / deltaT;
  errTprev = errT;
  float eintT = eintT + errT * deltaT;
  float PIDt = kp * errT + ki * eintT + kd * edervT;
  PIDt = fmaxf(-0.4, fminf(PIDt, 0.4));
  IN_Kinematic(ty, tx, PIDt);
}
void IN_Kinematic(float vx, float vy, float vt) {
  float v1 = (sin(torad(45 + hadap)) * vy + cos(torad(45 + hadap)) * vx + R_robot * vt) / k_roda;
  float v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
  v1Filt = v1;
  float v2 = (sin(torad(135 + hadap)) * vy + cos(torad(135 + hadap)) * vx + R_robot * vt) / k_roda;
  float v2Filt = 0.854 * v2Filt + 0.0728 * v2 + 0.0728 * v2Prev;
  v2Filt = v2;
  float v3 = (sin(torad(225 + hadap)) * vy + cos(torad(225 + hadap)) * vx + R_robot * vt) / k_roda;
  float v3Filt = 0.854 * v3Filt + 0.0728 * v3 + 0.0728 * v3Prev;
  v3Filt = v3;
  float v4 = (sin(torad(315 + hadap)) * vy + cos(torad(315 + hadap)) * vx + R_robot * vt) / k_roda;
  float v4Filt = 0.854 * v4Filt + 0.0728 * v4 + 0.0728 * v4Prev;
  v4Filt = v4;

  Serial3.print("!");
  Serial3.println(v1);
  Serial3.print("@");
  Serial3.println(v2);
  Serial3.print("#");
  Serial3.println(v3);
  Serial3.print("$");
  Serial3.println(v4);
}

void updateCMPS() {
  char tmp;  // Variabel temporary
  while (Serial2.available()) {
    tmp = Serial2.read();
    cmps.buffer[cmps.counter++] = tmp;
    if (tmp == '\n') {                                    // Langkah 1
      cmps.buffer[cmps.counter] = 0;                      // Karakter terminator
      cmps.heading = atof(strtok(cmps.buffer + 5, ","));  // Langkah 2-4
      cmps.counter = 0;
    }
  }
}

void calibrate_IMU() {
  Serial2.begin(115200);  // Serial GY25
  delay(3000);            // Jeda 3 detik

  // Kalibrasi Tilt
  Serial2.write(0xA5);
  Serial2.write(0x54);

  delay(1000);  // Jeda sebelum kalibrasi heading

  // Kalibrasi Heading
  Serial2.write(0xA5);
  Serial2.write(0x55);

  delay(100);  // Jeda sebelum konfigurasi output

  // Output ASCII
  Serial2.write(0xA5);
  Serial2.write(0x53);

  delay(100);  // Jeda sebentar
}
void readEncoder1() {
  int b = digitalRead(encB1);
  if (b > 0) {
    pos1++;
  } else {
    pos1--;
  }
}
void readEncoder2() {
  int b = digitalRead(encB2);
  if (b > 0) {
    pos2++;
  } else {
    pos2--;
  }
}
void readEncoder3() {
  int b = digitalRead(encB3);
  if (b > 0) {
    pos3++;
  } else {
    pos3--;
  }
}
void readEncoder4() {
  int b = digitalRead(encB4);
  if (b > 0) {
    pos4++;
  } else {
    pos4--;
  }
}
void setMotor(int dir, int pwm, int pinPWM, int pinCW, int pinCCW) {
  analogWrite(pinPWM, pwm);  //37.5
  if (dir == 1) {
    digitalWrite(pinCW, 1);
    digitalWrite(pinCCW, 0);
  } else if (dir == -1) {
    digitalWrite(pinCW, 0);
    digitalWrite(pinCCW, 1);
  } else if (dir == 2) {
    digitalWrite(pinCW, 1);
    digitalWrite(pinCCW, 1);
  } else {
    digitalWrite(pinCW, 1);
    digitalWrite(pinCCW, 0);
  }
}