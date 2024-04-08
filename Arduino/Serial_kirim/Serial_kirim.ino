
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial3.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("$");Serial.println("1");
  Serial3.print("$");Serial3.println("1");
  delay(1000);
  Serial3.print("$");Serial3.println("0");
  Serial.print("$");Serial.println("0");
  delay(1000);
}
