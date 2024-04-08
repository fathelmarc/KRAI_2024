float roll, pitch, yaw;
unsigned char Re_buf[8], counter = 0;
void setup()
{
  Serial.begin(9600);
  kalibrasi();
}

void loop() {
  parsing();
  Serial.print("roll= ");
  Serial.print(roll);
  Serial.print(" pitch= ");
  Serial.print(pitch);
  Serial.print(" yaw= ");
  Serial.print(yaw);
  Serial.println();
}
void kalibrasi() {
  Serial3.begin(115200);
  delay(3000);
  Serial3.write(0xA5);
  Serial3.write(0x54);
  delay(1000);
  Serial3.write(0xA5);
  Serial3.write(0x55);
  delay(100);
}
void parsing() {
  Serial3.write(0XA5);
  Serial3.write(0X51);//send it for each read
  while (Serial3.available()) {
    Re_buf[counter] = (unsigned char)Serial3.read();
    if (counter == 0 && Re_buf[0] != 0xAA) return;
    counter++;
    if (counter == 8)
    {
      counter = 0;
      if (Re_buf[0] == 0xAA && Re_buf[7] == 0x55) // data package is correct
      {
        yaw = (int16_t)(Re_buf[1] << 8 | Re_buf[2]) / 100.00;
        pitch = (int16_t)(Re_buf[3] << 8 | Re_buf[4]) / 100.00;
        roll = (int16_t)(Re_buf[5] << 8 | Re_buf[6]) / 100.00;
      }
    }
  }
}
