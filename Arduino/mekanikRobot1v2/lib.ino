void kirim() {
  parsing();
  Udp.beginPacket(IPAddress(192, 168, 0, 7), 5555);
  imu_pure = imu.yaw * -1;
  Udp.print(imu_pure);//0
  Udp.print(",");
  gyro = imu.yaw;
  if (gyro > 0) {
    float selisih = 180 - gyro;
    gyro = 180 + selisih;
    if (gyro > 360) {
      gyro = 0;
    }
  }
  if (gyro < 0) {
    gyro = gyro * -1;
  }
  Udp.print(gyro);//1
  Udp.print(",");
  Udp.print(imu.pitch);//2
  Udp.print(",");
  Udp.print(imu.roll);//3
  Udp.println();
  Udp.endPacket();
  Udp.flush();
}
void launcher(float pwm_valL, float pwm_valR) {
  analogWrite(PWM_right, abs(pwm_valR));
  analogWrite(PWM_left, abs(pwm_valL));

  if (pwm_valR < 0) {
    digitalWrite(CW_right, 1);
    digitalWrite(CCW_right, 0);
  } else if (pwm_valR > 0) {
    digitalWrite(CW_right, 0);
    digitalWrite(CCW_right, 1);
  }

  if (pwm_valL < 0) {
    digitalWrite(CW_left, 1);
    digitalWrite(CCW_left, 0);
  } else if (pwm_valL > 0) {
    digitalWrite(CW_left, 0);
    digitalWrite(CCW_left, 1);
  }
}

void Launcher(float pwm_val){
  analogWrite(PWM_launch, fabs(pwm_val));
  if (pwm_val < 0) {
    digitalWrite(CW_left, 1);
    digitalWrite(CCW_left, 0);
  } else if (pwm_val > 0) {
    digitalWrite(CW_left, 0);
    digitalWrite(CCW_left, 1);
  } else {
    digitalWrite(CW_launch, 0);
    digitalWrite(CCW_launch,0);
  }
}

void terima() {
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    if (packetSize >= sizeof(packetBuffer) - 1) {
      packetSize = sizeof(packetBuffer) - 1;
    }
    Udp.read(packetBuffer, packetSize);
    packetBuffer[packetSize] = '\0'; // Null-terminate the string
    sign = atoi(packetBuffer);
  }
}

void calibration() {
  Serial3.begin(115200); // Serial GY25
  delay(3000); // Jeda 3 detik

  // Kalibrasi Tilt
  Serial3.write(0xA5);
  Serial3.write(0x54);

  delay(1000); // Jeda sebelum kalibrasi heading

  // Kalibrasi Heading
  Serial3.write(0xA5);
  Serial3.write(0x55);

  delay(100); // Jeda sebelum konfigurasi output

  // Output ASCII
  Serial3.write(0xA5);
  Serial3.write(0x53);

  delay(100); // Jeda sebentar
}

void parsing() {
  Serial3.write(0XA5);
  Serial3.write(0X51);//send it for each read
  while (Serial3.available()) {
    imu.buffer[imu.counter] = (unsigned char)Serial3.read();
    if (imu.counter == 0 && imu.buffer[0] != 0xAA) return;
    imu.counter++;
    if (imu.counter == 8)
    {
      imu.counter = 0;
      if (imu.buffer[0] == 0xAA && imu.buffer[7] == 0x55) // data package is correct
      {
        imu.yaw = (int16_t)(imu.buffer[1] << 8 | imu.buffer[2]) / 100.00;
        imu.pitch = (int16_t)(imu.buffer[3] << 8 | imu.buffer[4]) / 100.00;
        imu.roll = (int16_t)(imu.buffer[5] << 8 | imu.buffer[6]) / 100.00;
      }
    }
  }
}

void changer() {
  if (sign == 101) {
    lamp = 1;
  } else if (sign == 102) {
    lamp = 2;
  } else if (sign == 103) {
    lamp = 3;
  } else if (sign == 104) {
    lamp = 4;
  }
  switch (lamp) {
    case 1:
      digitalWrite(lamp1, 1);
      digitalWrite(lamp2, 0);
      digitalWrite(lamp3, 0);
      digitalWrite(lamp4, 0);
      break;
    case 2:
      digitalWrite(lamp1, 0);
      digitalWrite(lamp2, 1);
      digitalWrite(lamp3, 0);
      digitalWrite(lamp4, 0);
      break;
    case 3:
      digitalWrite(lamp1, 0);
      digitalWrite(lamp2, 0);
      digitalWrite(lamp3, 1);
      digitalWrite(lamp4, 0);
      break;
    case 4:
      digitalWrite(lamp1, 0);
      digitalWrite(lamp2, 0);
      digitalWrite(lamp3, 0);
      digitalWrite(lamp4, 1);
      break;
  }
}
