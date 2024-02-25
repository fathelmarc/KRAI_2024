void kirim() {
  updateCMPS();
  int imu = cmps.heading;
  imu = imu * -1;
  Udp.beginPacket(IPAddress(192, 168, 0, 7), 5555);
  Udp.println(imu);
  Serial.println(imu);
  Udp.endPacket();
  Udp.flush();
}
void terima() {
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    if (packetSize >= sizeof(packetBuffer) - 1) {
      packetSize = sizeof(packetBuffer) - 1;
    }
    Udp.read(packetBuffer, packetSize);
    packetBuffer[packetSize] = '\0'; // Null-terminate the string
    sign = packetBuffer;
  }
}
void motorT(int pwmval) {
  pwmval = max(-150, min(pwmval, 150));
  analogWrite(PWMT, fabs(pwmval));

  if (pwmval < 0) {
    digitalWrite(CWT, 1);
    digitalWrite(CCWT, 0);
  }
  else if (pwmval > 0) {
    digitalWrite(CWT, 0);
    digitalWrite(CCWT, 1);
  }
  else {
    digitalWrite(CWT, 0);
    digitalWrite(CCWT, 0);
  }
}
void motorK(int pwmval) {
  pwmval= max(-60, min(pwmval, 60));
  analogWrite(PWMK, fabs(pwmval));

  if (pwmval < 0) {
    digitalWrite(CWK, 1);
    digitalWrite(CCWK, 0);
  }
  else if (pwmval > 0) {
    digitalWrite(CWK, 0);
    digitalWrite(CCWK, 1);
  }
  else {
    digitalWrite(CWK, 0);
    digitalWrite(CCWK, 0);
  }
}
void setMotor(int dir, int pwmval, int cw, int ccw, int pwmpin) {
  analogWrite(pwmpin, pwmval);
  if (dir == 1) {
    digitalWrite(cw, HIGH);
    digitalWrite(ccw, LOW);
  } else if (dir == -1) {
    digitalWrite(cw, LOW);
    digitalWrite(ccw, HIGH);
  } else if (dir == 2) {
    digitalWrite(cw, HIGH);
    digitalWrite(ccw, HIGH);
  } else {
    digitalWrite(cw, LOW);
    digitalWrite(ccw, LOW);
  }
}

void kalibrasi() {
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
void updateCMPS() {
  char tmp; // Variabel temporary
  while (Serial3.available()) {
    tmp = Serial3.read();
    cmps.buffer[cmps.counter++] = tmp;
    if (tmp == '\n') { // Langkah 1
      cmps.buffer[cmps.counter] = 0; // Karakter terminator
      cmps.heading = atof(strtok(cmps.buffer + 5, ",")); // Langkah 2-4
      cmps.counter = 0;
    }
  }
}
