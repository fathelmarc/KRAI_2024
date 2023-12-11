void updateVel() {
  char tmp; // Variabel temporary
  while (Serial.available()) {
    tmp = Serial.read();
    motor.buffer[motor.counter++] = tmp;
    if (tmp == '\n') { // Langkah 1
      motor.buffer[motor.counter] = 0; // Karakter terminator
      if (motor.buffer[0] == '#') {
        motor.kecepatan = atof(strtok(motor.buffer + 1, '='));
      }
      motor.counter = 0;
    }
  }
}
void PID_kec(float v_target) {
  //time
  currT = micros();//hitung waktu dalam micro secondt
  deltaT = ((float)(currT - prevT)) / 1.0e6; //convert ke detik
  prevT = currT;
  int enc;
  noInterrupts();
  enc = pos;
  interrupts();

  //calc to rad/s
  radian = (enc - encPrev) / deltaT;
  encPrev = enc;
  rads = radian / (200 * 19.2);

  //low pass filter
  radsFilt = 0.854 * radsFilt + 0.0728 * rads + 0.0728 * radsPrev;
  radsPrev = rads;

  //constant
  kp = 400;//7.25;//2; nilai lebih baik KP< KI untuk motor 2
  ki = 10;//0.0179063;//0.081627717;
  kd = 0;//0.000025;//2.64616;

  //error
  e = v_target - radsFilt;

  //integral
  eintegral = eintegral + e * deltaT;

  //derivative
  ederivative = (e - eprev) / deltaT;
  eprev = e;

  //control signal
  u = kp * e + ki * eintegral + kd * ederivative;

  int dir = 1;
  if (u < 0) {
    dir = -1;
  }
  float pwr = fabs(u);
  if (pwr > 150) {
    pwr = 150;
  }
  setMotor(dir, pwr);
}

void setMotor(int dir, int pwmVal) {
  analogWrite(PWM, pwmVal);
  if (dir == -1) {
    digitalWrite(In1, 1);
    digitalWrite(In2, 0);
  } else if (dir == 1) {
    digitalWrite(In1, 0);
    digitalWrite(In2, 1);
  } else {
    digitalWrite(In1, 0);
    digitalWrite(In2, 0);
  }
}
void readencoder() {
  int b = digitalRead(encB);

  if (b > 0) {
    increment = 1;
  } else {
    increment = -1;
  }
  pos = pos + increment;
}
