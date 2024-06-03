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
  switch (Ethernet.linkStatus()) {
    case LinkON:
      Serial.println("Ethernet link status: LINK ON");
      encSend();
      float pwmVal = PWMReceive();
      setMotor(pwmVal);
      break;
    case LinkOFF:
      Serial.println("Ethernet link status: LINK OFF");
      setMotor(0);
      break;
    default:
      Serial.println("Ethernet link status: UNKNOWN");
      break;
  }
}
