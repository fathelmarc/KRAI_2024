
void readencoder() {
  int b = digitalRead(encB);
  int increment = 0;
  if (b > 0) {
    increment = 1;
  } else {
    increment = -1;
  }
  pos += increment;
}

float PWMReceive() {
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    if (packetSize >= sizeof(packetBuffer) - 1) {
      packetSize = sizeof(packetBuffer) - 1;
    }
    Udp.read(packetBuffer, packetSize);
    packetBuffer[packetSize] = '\0'; // Null-terminate the string
    return atof(packetBuffer);
  }
}
void encSend() {
  Udp.beginPacket(serverIP, localPort);
  Udp.write(pos);
  Udp.endPacket();
  Udp.flush();
}

void setMotor(int pwmVal) {
  analogWrite(PWM, fabs(pwmVal));
  if (pwmVal <= -1) {
    digitalWrite(In1, 1);
    digitalWrite(In2, 0);
  } else if (pwmVal >= 1) {
    digitalWrite(In1, 0);
    digitalWrite(In2, 1);
  } else {
    digitalWrite(In1, 0);
    digitalWrite(In2, 0);
  }
}
