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

IPAddress ip(192, 168, 0, 68);//ip address arduino
IPAddress gateway(192, 168, 0, 2);//gateway
IPAddress subnet(255, 255, 255, 0); //netmask
IPAddress myDns(8, 8, 8, 8); //tak penting mungkin

IPAddress serverIP(192, 168, 0, 7);//ip address laptop

unsigned int localPort = 5555;
char packetBuffer[512];

EthernetUDP Udp;

//globals
volatile unsigned long prevT , currT;
int encPrev,i;
volatile long pos;
volatile float e, eintegral, u, ederivative, eprev;
float radsFilt, radsPrev, deltaT, kp, ki, kd, rads, radian, kecepatan;
int32_t freq = 35;
void setup() {
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
  encSend();
  speedReceive();
  if (kecepatan > 5) {
    PID_kec(5);
  } else if (kecepatan < -5) {
    PID_kec(-5);
  } else if (kecepatan < 5 || kecepatan > -5) {
    PID_kec(kecepatan);
  } else {
    PID_kec(0);
  }
  delay(1);
}

void speedReceive() {
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    if (packetSize >= sizeof(packetBuffer) - 1) {
      packetSize = sizeof(packetBuffer) - 1;
    }
    Udp.read(packetBuffer, packetSize);
    packetBuffer[packetSize] = '\0'; // Null-terminate the string
    kecepatan = atof(packetBuffer);
  }
}
void encSend() {
  Udp.beginPacket(IPAddress(192, 168, 0, 7), localPort);
  Udp.println(pos);
  Udp.endPacket();
  Udp.flush();
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

  //constant value of PID
  kp = 400;//7.25;//2; nilai lebih baik KP< KI untuk motor 2
  ki = 40;//0.0179063;//0.081627717;
  kd = 0;//0.000025;//2.64616;

  //error
  e = v_target - radsFilt;

  //integral
  eintegral += e * deltaT;

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
  if (radsFilt <= 0.05 && radsFilt >= -0.05 && v_target ==0) {
    analogWrite(PWM, 0);
    digitalWrite(In1, 0);
    digitalWrite(In2, 0);
  }
  Serial.print(radsFilt);
  Serial.print(" ");
  Serial.println(v_target);
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
  int increment = 0;
  if (b > 0) {
    increment = 1;
  } else {
    increment = -1;
  }
  pos += increment;
}
