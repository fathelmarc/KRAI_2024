#include <Servo.h>
int IR1 = A5;
int IR2 = A2;
int pwm1 = 6;
int pwm2 = 9;
int motor1 = 5;
int motor2 = 4;
int motor3 = 8;
int motor4 = 7;
Servo myservo, servo1;
void setup() {
  Serial.begin(115200);
  pinMode(3, INPUT);
  digitalWrite(3, HIGH);
  myservo.attach(10);
  servo1.attach(11);
  
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);
}
void loop() {
  // put your main code here, to run repeatedly:
  int switchstate = digitalRead(3);

  if (digitalRead(IR1) == LOW) {
    servo1kunci();
    motor1naik();
    delay(100);
    motor1mati();
    while (true) {
      switchstate = digitalRead(3);
      if (switchstate == LOW) {
        servo1buka();
        motor1turun();
        delay(1);
        motor1mati();
        break;
      }
    }
  }
  if (digitalRead(IR2) == LOW) {
    servo2kunci();
    motor2naik();
    delay(100);
    motor2mati();
    while (true) {
      switchstate = digitalRead(3);
      if (switchstate == LOW);
      servo2buka();
      motor2turun();
      delay(1);
      motor2mati();
      break;
    }
  }
}
void servo1buka() {
  myservo.write(90);
  delay(1000);
}
void servo1kunci() {
  myservo.write(140);
}
void servo2buka() {
  servo1.write(10);
  delay(1000);
}

void servo2kunci() {
  servo1.write(130);
}
void motor1mati() {
  analogWrite(pwm2, 0);
  digitalWrite(motor3, LOW);
  digitalWrite(motor4, LOW);
  delay(100);
}
void motor1turun() {
  analogWrite(pwm2, 255);
  digitalWrite(motor3, HIGH);
  digitalWrite(motor4, LOW);
  delay(1000);
}
void motor1naik() {
  analogWrite(pwm2, 255);
  digitalWrite(motor3, LOW);
  digitalWrite(motor4, HIGH);
  delay(1400);
}
void motor2mati() {
  analogWrite(pwm1, 0);
  digitalWrite(motor1, LOW);
  digitalWrite(motor2, LOW);
  delay(100);
}
void motor2naik() {
  analogWrite(pwm1, 255);
  digitalWrite(motor1, LOW);
  digitalWrite(motor2, HIGH);
  delay(1400);
}
void motor2turun() {
  analogWrite(pwm1, 255);
  digitalWrite(motor1, HIGH);
  digitalWrite(motor2, LOW);
  delay(1000);
}
