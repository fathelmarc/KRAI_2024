#include "control.h"
#include <Encoder.h>

//atur nilai kp ki kd untuk menyesuaikan kecepatan
float kp = 18;
float ki = 6;
float kd = 0;
Controller roller_left(kp, ki, kd);  //bisa langsung tembak angka disini
Controller roller_right(kp, ki, kd);

Encoder encoder_left(10, 9);    //pin encoder kiri
Encoder encoder_right(16, 17);  //pin encoder kanan

const int cw[2] = { 37, 23 };  // array pertama adalah kanan dan array ke 2 adalah kiri
const int ccw[2] = { 36, 22 };

// definisikan PWM maximum value dan frequency
const int pwm_bits = 8;
const int pwm_max = (1 << pwm_bits) - 1;
const float pwm_frequency = 18000.0;

float prevT = 0.0;

void setup() {
  for (int i = 0; i < 2; i++) {
    pinMode(cw[i], OUTPUT);
    pinMode(ccw[i], OUTPUT);
    analogWriteFrequency(cw[i], pwm_frequency);
    analogWriteFrequency(ccw[i], pwm_frequency);
    analogWriteResolution(pwm_bits);
    analogWrite(cw[i], 0);
    analogWrite(ccw[i], 0);
  }
}
void motion(float pwm_value_right, float pwm_value_left) {
  roller_left.setMotor(cw[0], ccw[0], pwm_value_right);
  roller_right.setMotor(cw[1], ccw[1], pwm_value_left);
}

unsigned long prevTime = 0;
const long interval = 0;

float speed_target;
void loop() {
//  hitung waktu sekarang
  unsigned long currTime = millis();
  if(currTime - prevTime >= interval){
    prevTime = currTime;
    Serial.print("time:");
    Serial.print(currTime / 100);
    Serial.print("s,");
  }
  
  //untuk berinteraksi dengan serial monitor
  //jadi tinggal masukkan nilai misalnya 8 atau berapa gitu
  if(Serial.available()){
    char data_char = Serial.read();
    if(data_char >= '0' && data_char <= '9'){
      float data = data_char - '0';
      speed_target = data;
    }
  }

  // bisa juga langsung tembak nilai seperti dibawah ini
  // speed_target = 8 atau berapa
  
  // buat hitungan delta diluar class agar sesuai dengan sistem
  float currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  prevT = currT;

  // float speed_target = data;  // sesuaikan dengan target kecepatan yang diinginkan
  float read_encoder_left = encoder_left.read();
  float read_encoder_right = encoder_right.read();

  //hitung pid launcher kanan
  float control_right = roller_right.control(speed_target, read_encoder_right, deltaT);
  //hitung pid launcher kiri & untuk sebelah kiri buat minus saja
  float control_left = roller_left.control(-speed_target, read_encoder_left, deltaT);
  
  motion(control_right, control_left);  //input hasil kecepatan pwm, pertama kanan kedua kiri
  //print nilai sesuai yang dibutuhkandl
  Serial.print(-speed_target);  //untuk perbandingan dengan target
  Serial.print(",");
  Serial.print(roller_left.getFilteredValue());  //getPureValue() untuk nilai tanpa filter
  Serial.print(",");
  Serial.print(speed_target);
  Serial.print(",");
  Serial.print(roller_right.getFilteredValue());  //getPureValue() untuk nilai tanpa filter
  Serial.println();
  
  delay(100);
}
