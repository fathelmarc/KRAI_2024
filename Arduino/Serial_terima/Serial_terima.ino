char packetBuffer[512];
struct PG45 {
  char buffer[50];
  int counter;
  float vel,vel1;
} base;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void updateVel() {
  char tmp; // Variabel temporary
  while (Serial.available()) {
    tmp = Serial.read();
    base.buffer[base.counter++] = tmp;
    if (tmp == '\n') { // Langkah 1
      base.buffer[base.counter] = 0; // Karakter terminator
      if (base.buffer[0] == '$') {
        base.vel = atof(strtok(base.buffer + 1, '='));
      }
      base.counter = 0;
    }
  }
}

void loop() {
  updateVel();
  Serial.print(base.vel);
  Serial.println();
}
