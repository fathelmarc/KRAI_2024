#include <Servo.h>
#define limit1A A2
#define limit2A A3
#define limit1B A4
#define limit2B A5
#define IR1 A0
#define IR2 A1
#define but1 4
#define but2 5

//actuator
#define cw1 13
#define ccw1 12
#define pwm1 11

#define cw2 7
#define ccw2 8
#define pwm2 6

Servo gp1, gp2;

void setup() {
  Serial.begin(115200);
  pinMode(but1, INPUT_PULLUP);
  pinMode(but2, INPUT_PULLUP);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(limit1A, INPUT);
  pinMode(limit2A, INPUT);
  pinMode(limit1B, INPUT);
  pinMode(limit2B, INPUT);

  pinMode(cw1, OUTPUT);
  pinMode(ccw1, OUTPUT);
  pinMode(pwm1, OUTPUT);

  pinMode(cw2, OUTPUT);
  pinMode(ccw2, OUTPUT);
  pinMode(pwm2, OUTPUT);

  gp1.attach(10);
  gp2.attach(9);
}
int opn1 = 110;
void loop() {
  int trig1 = digitalRead(but1);
  int trig2 = digitalRead(but2);
  int lim1A = digitalRead(limit1A);
  int lim2A = digitalRead(limit2A);
  int lim1B = digitalRead(limit1B);
  int lim2B = digitalRead(limit2B);
  int sens1 = digitalRead(IR1);
  int sens2 = digitalRead(IR2);
  int gp1Con = gp1.read();
  int gp2Con = gp2.read();
  while (sens1 == 0 && lim1A == 0 && trig1 == 1) {
    gp1.write(110);
    setMotor(1, 255, cw1, ccw1, pwm1);
    Serial.println("naik1");
    break;
  } while (trig1 == 0 && lim1B == 0) {
    gp1.write(0);
    setMotor(-1, 255, cw1, ccw1, pwm1);
    Serial.println("turun1");
    break;
  } 
  
  while (sens2 == 0 && lim2A == 0 && trig2 == 1) {
    gp2.write(85);
    setMotor(1,255,cw2,ccw2,pwm2);
    Serial.println("naik2");
    break;
  } while (trig2 == 0 && lim2B == 0) {
    gp2.write(0);
    setMotor(-1, 255, cw2, ccw2, pwm2);
    Serial.println("turun2");
    break;
  } 
  
  
  while (lim1A == 1) {
    Serial.println("mati1up");
    setMotor(0, 0, cw1, ccw1, pwm1);
    break;
  } while (lim2A == 1) {
    Serial.println("mati2up");
    setMotor(0, 0, cw2, ccw2, pwm2);
    break;
  } 
  
  while (lim1B == 1&& sens1 == 1 || trig1 == 0) {
    Serial.println("mati1down");
    gp1.write(0);
    setMotor(0, 0, cw1, ccw1, pwm1);
    break;
  } while (lim2B == 1 && sens2 == 1 || trig2 == 0) {
    Serial.println("mati2down");
    gp2.write(0);
    setMotor(0, 0, cw2, ccw2, pwm2);
    break;
  }
}

void setMotor(int dir, int pulse, int CW, int CCW, int PWM) {
  analogWrite(PWM, pulse);
  if (dir == 1) {
    digitalWrite(CW, HIGH);
    digitalWrite(CCW, LOW);
  } if (dir == -1) {
    digitalWrite(CW, LOW);
    digitalWrite(CCW, HIGH);
  } if (dir == 2) {
    digitalWrite(CW, HIGH);
    digitalWrite(CCW, HIGH);
  } if (dir == 0) {
    digitalWrite(CW, LOW);
    digitalWrite(CCW, LOW);
  }
}
