#define encA 3
#define encB 4
#define PWM 9
#define In1 7
#define In2 8

//globals
volatile unsigned long prevT , currT;
int encPrev ,increment;
volatile int pos;
volatile float e, eintegral, u, ederivative, eprev;
float radsFilt, radsPrev, deltaT, kp, ki, kd, rads, radian;

struct PG45 {
  char buffer[50];
  int counter;
  float vel, posisi, kecepatan,maxvel;
} motor;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  pinMode(encA, INPUT);
  pinMode(encB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encA), readencoder, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  updateVel();
  float minkec = motor.kecepatan;
    if (minkec>5){
    PID_kec(5);
    }else if (minkec<-5){
      PID_kec(-5);
    }else if(minkec<5 || minkec>-5){
      PID_kec(motor.kecepatan);
    }else {
      PID_kec(0);
    }
}
