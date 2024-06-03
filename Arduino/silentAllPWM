#include <Ethernet.h>
#include <EthernetUdp.h>
#define encA 3
#define encB 4
#define PWM 6
#define In1 7
#define In2 8
//69 310, 30, 0
//68 400, 40, 0
//67 400, 10, 0
//66 300, 30, 0
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

IPAddress ip(192, 168, 0, 66);//ip address arduino
IPAddress gateway(192, 168, 0, 4);//gateway
IPAddress subnet(255, 255, 255, 0); //netmask
IPAddress myDns(8, 8, 8, 8); //tak penting mungkin

IPAddress serverIP(192, 168, 0, 7);//ip address laptop

unsigned int localPort = 5555;
char packetBuffer[512];

EthernetUDP Udp;

volatile long pos;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(encA, INPUT);
  pinMode(encB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encA), readencoder, RISING);
  Ethernet.begin(mac, ip, myDns, gateway, subnet);
  Udp.begin(localPort);
}

void loop() {
  // put your main code here, to run repeatedly:
  encSend();
  float pwmVal = PWMReceive();
  setMotor(pwmVal);
}

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
