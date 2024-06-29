#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Empty.h>
#include <Arduino.h>
#include <Encoder.h>
//launcher 21 20
//jepit bola 28
//lifterBall 27
//liftest 26
#define WATCHDOG_TIMEOUT 1000
#define HEARTBEAT_TIMEOUT 2000
#define SILANG 0
#define TRIANGLE 2
const int CIRCLE = 1;
const int CIRCLE1 = -1;
const int SQUARE1 = -3;
const int SQUARE = 3;
#define L1 4
#define R1 5
#define L2 6
#define R2 7
#define Start 31

#define tutup 0
#define buka 1

#define angkat 1
#define turun 0

#define extend 1
#define shorten 0

const int ballGrip = 28;
const int ballLifter = 27;
const int ballLiftest = 26;

const int gripper1 = 31;//31
const int gripper2 = 32;//32
const int lifter = 29;
const int extender = 30;//30
const int launcherOut = 36;
const int launcherIn = 37;

int i;



class PIDController {
  private:
    float prevT;
    float KP, KI, KD;
    float encPrev;
    float radsPrev;
    float eintegral;
    float eprev;

  public:

    float radsFilt;
    PIDController(float kp_, float ki_, float kd_) : KP(kp_), KI(ki_), KD(kd_) {
      prevT = 0.0;
      encPrev = 0.0;
      radsPrev = 0.0;
      radsFilt = 0.0;
      eintegral = 0.0;
      eprev = 0.0;
    }

    float control(float target, float enc, float deltaT) {
      float radian = (enc - encPrev) / deltaT;
      encPrev = enc;
      float rads = radian / (200 * 19.2);

      radsFilt = 0.854 * radsFilt + 0.0728 * rads + 0.0728 * radsPrev;
      radsPrev = rads;

      float e = target - radsFilt;
      eintegral += e * deltaT;
      float ederivative = (e - eprev) / deltaT;
      eprev = e;

      float u = KP * e + KI * eintegral + KD * ederivative;
      return u;
    }
    float getFilteredValue() const {
      return radsFilt;
    }
};
float Kp = 18;
float Ki = 4;
float Kd = 0;
PIDController motor1_pid(Kp, Ki, Kd);
PIDController motor2_pid(Kp, Ki, Kd);
PIDController motor3_pid(Kp, Ki, Kd);
PIDController motor4_pid(Kp, Ki, Kd);


// Encoder declarations
Encoder enc1(14, 15);
Encoder enc3(17, 16);
Encoder enc2(11, 12);
Encoder enc4(9, 10);


ros::NodeHandle nh;
std_msgs::Int32MultiArray rollPitch_msg;
ros::Publisher pubRP("rollPitch", &rollPitch_msg);


std_msgs::Int32MultiArray enc_msg;
ros::Publisher pub("encoder", &enc_msg);

std_msgs::Int32MultiArray sendIMU;
ros::Publisher pubIMU("IMU", &sendIMU);




unsigned long last_heartbeat_time = 0;
void heartbeatCallback(const std_msgs::Empty& msg) {
  last_heartbeat_time = millis();
}
ros::Subscriber<std_msgs::Empty> sub_heartbeat("heartbeat", heartbeatCallback);




float target1 = 0, target2 = 0, target3 = 0, target4 = 0;
void callBack(const std_msgs::Float32MultiArray& msg) {
  target1 = msg.data[0];
  target2 = msg.data[1];
  target3 = msg.data[2];
  target4 = msg.data[3];
}
ros::Subscriber<std_msgs::Float32MultiArray> sub("speed_motor", &callBack);


int sign = 0;
void arrayCallback(const std_msgs::Int32MultiArray& enc_msg) {
  sign = enc_msg.data[0];
}
ros::Subscriber<std_msgs::Int32MultiArray> subCommand("command", &arrayCallback);




// Motor control pins
const int cw1 = 18, ccw1 = 19, cw2 = 5, ccw2 = 6;
const int cw3 = 22, ccw3 = 23, cw4 = 3, ccw4 = 4;

// Define the PWM maximum value and frequency
const int pwm_bits = 8, pwm_max = (1 << pwm_bits) - 1;
const float pwm_frequency = 18000;


//imu declaration
int roll, pitch, yaw;
unsigned char Re_buf[8], counter = 0;

void setup() {
  nh.initNode();

  nh.advertise(pubRP);
  rollPitch_msg.data_length = 2;
  rollPitch_msg.data = (int32_t*)malloc(sizeof(int32_t) * rollPitch_msg.data_length);

  nh.advertise(pub);
  enc_msg.data_length = 5;
  enc_msg.data = (int32_t*)malloc(sizeof(int32_t) * enc_msg.data_length);

  nh.advertise(pubIMU);
  sendIMU.data_length = 2;
  sendIMU.data = (int32_t*)malloc(sizeof(int32_t) * sendIMU.data_length);

  nh.subscribe(sub);
  nh.subscribe(subCommand);
  nh.subscribe(sub_heartbeat);
  last_heartbeat_time = millis();

  kalibrasi();
  pinMode(ballGrip, OUTPUT);
  pinMode(ballLifter, OUTPUT);
  pinMode(ballLiftest, OUTPUT);
  pinMode(gripper1, OUTPUT);
  pinMode(gripper2, OUTPUT);
  pinMode(lifter, OUTPUT);
  pinMode(extender, OUTPUT);
  pinMode(cw1, OUTPUT); pinMode(ccw1, OUTPUT);
  pinMode(cw2, OUTPUT); pinMode(ccw2, OUTPUT);
  pinMode(cw3, OUTPUT); pinMode(ccw3, OUTPUT);
  pinMode(cw4, OUTPUT); pinMode(ccw4, OUTPUT);
  pinMode(launcherIn, OUTPUT);
  pinMode(launcherOut, OUTPUT);

  analogWriteFrequency(launcherIn, pwm_frequency);
  analogWriteFrequency(launcherOut, pwm_frequency);
  analogWriteFrequency(cw1, pwm_frequency);
  analogWriteFrequency(ccw1, pwm_frequency);
  analogWriteFrequency(cw2, pwm_frequency);
  analogWriteFrequency(ccw2, pwm_frequency);
  analogWriteFrequency(cw3, pwm_frequency);
  analogWriteFrequency(ccw3, pwm_frequency);
  analogWriteFrequency(cw4, pwm_frequency);
  analogWriteFrequency(ccw4, pwm_frequency);

  analogWriteResolution(pwm_bits);
  analogWrite(launcherIn, 0);
  analogWrite(launcherOut, 0);
  analogWrite(cw1, 0); analogWrite(ccw1, 0);
  analogWrite(cw2, 0); analogWrite(ccw2, 0);
  analogWrite(cw3, 0); analogWrite(ccw3, 0);
  analogWrite(cw4, 0); analogWrite(ccw4, 0);

  digitalWrite(extender, shorten);
  digitalWrite(gripper1, tutup);
  digitalWrite(gripper2, tutup);
  digitalWrite(lifter, turun);
}

void motion(float pwmVal_1, float pwmVal_2, float pwmVal_3, float pwmVal_4) {
  setMotor(cw1, ccw1, pwmVal_1);
  setMotor(cw2, ccw2, pwmVal_2);
  setMotor(cw3, ccw3, pwmVal_3);
  setMotor(cw4, ccw4, pwmVal_4);
  if (fabs(motor1_pid.getFilteredValue()) <= 1 && fabs(target1) == 0) {
    setMotor(cw1, ccw1, 0);
  }
  if (fabs(motor2_pid.getFilteredValue()) <= 1 && fabs(target2) == 0) {
    setMotor(cw2, ccw2, 0);
  }
  if (fabs(motor3_pid.getFilteredValue()) <= 1 && fabs(target3) == 0) {
    setMotor(cw3, ccw3, 0);
  }
  if (fabs(motor4_pid.getFilteredValue()) <= 1 && fabs(target4) == 0) {
    setMotor(cw4, ccw4, 0);
  }
}

void setMotor(int cwPin, int ccwPin, float pwmVal) {
  float lim_pwm = fmax(-200 , fmin(pwmVal, 200));
  if (lim_pwm > 0) {
    analogWrite(cwPin, fabs(lim_pwm));
    analogWrite(ccwPin, 0);
  } else if (lim_pwm < 0) {
    analogWrite(cwPin, 0);
    analogWrite(ccwPin, fabs(lim_pwm));
  } else {
    analogWrite(cwPin, 0);
    analogWrite(ccwPin, 0);
  }
}
int motorStart = 0;
float prevT;
void loop() {
  float currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  if (deltaT <= 0.0) {
    return 0.0;
  }
  prevT = currT;
  changer();
  if (motorStart == 1) {
    analogWrite(launcherIn, 255);
    analogWrite(launcherOut, 0);
  } else {
    analogWrite(launcherIn, 0);
    analogWrite(launcherOut, 0);
  }

  float controlVal1 = motor1_pid.control(target4, enc1.read(), deltaT); //4
  float controlVal2 = motor2_pid.control(target1, enc2.read(), deltaT); //1
  float controlVal3 = motor3_pid.control(target2, enc3.read(), deltaT); //2
  float controlVal4 = motor4_pid.control(target3, enc4.read(), deltaT); //3
  motion(controlVal1, controlVal2, controlVal3, controlVal4);
  if (millis() - last_heartbeat_time > HEARTBEAT_TIMEOUT) {
    motion(0, 0, 0, 0); // Stop the motor if the heartbeat has timed out
  }

  enc_msg.data[0] = enc4.read();
  enc_msg.data[1] = enc1.read();
  enc_msg.data[2] = enc2.read();
  enc_msg.data[3] = enc3.read();

  sendingIMU();
  pubRP.publish(&rollPitch_msg);
  pubIMU.publish(&sendIMU);
  pub.publish(&enc_msg);
  nh.spinOnce();

  delay(5);
}

void stopMotor() {
  analogWrite(cw1, 0);
  analogWrite(ccw1, 0);
  analogWrite(cw2, 0);
  analogWrite(ccw2, 0);
  analogWrite(cw3, 0);
  analogWrite(ccw3, 0);
  analogWrite(cw4, 0);
  analogWrite(ccw4, 0);
}
#define releasing 0
#define gripping 1
#define liftBall 1
#define dropBall 0
#define liftHighBall 1
#define dropLowBall 0
int count_close_both = 0;
int open_first = 0;
int lift_second = 0;
int ball_delay_lifting = 0;
int ball_delay_liftest = 0;
int ball_delay_droping = 0;
int ball_delay_dropest = 0;

int seqFL1 = 0;
int seqFL2 = 0;
int seq1 = 0;
int seq2 = 0;
int seq3 = 0;
void changer() {
  if (sign == 64) {
    motorStart = 1;
    digitalWrite(ballLiftest, liftHighBall);
    delay(800);
    digitalWrite(ballGrip, releasing);

  } else if (sign == 46) {
    motorStart = 0;
  } else if (sign == L1) {
    digitalWrite(lifter, angkat);
  } else if (sign == R1) {
    digitalWrite(lifter, turun);
  } else if (sign == 21) {
    digitalWrite(extender, shorten);
    digitalWrite(gripper1, tutup);
    digitalWrite(gripper2, tutup);
  } else if (sign == 31) {
    digitalWrite(extender, extend);
    digitalWrite(gripper1, buka);
    digitalWrite(gripper2, buka);
    seq1 = 1;
  } else if (sign == SQUARE && seq1 == 1) {

    digitalWrite(gripper1, tutup);
    digitalWrite(gripper2, tutup);
    delay(500);
    digitalWrite(lifter, angkat);
    seq1 = 0;
    seq2 = 1;
    seq3 = 1;
  } else if (sign == CIRCLE && seq2 == 1) {

    count_close_both = 0;
    digitalWrite(lifter, turun);
    delay(500);
    digitalWrite(gripper1, buka);
    seq1 = 1;
    seq2 = 0;
    seq3 = 1;
  } else if (sign == CIRCLE1 && seq3 == 1) {

    digitalWrite(lifter, turun);
    delay(500);
    digitalWrite(gripper2, buka);
    seq1 = 1;
    seq2 = 1;
    seq3 = 0;
  }
  if (sign == 90) {
    seqFL1 = 0;
    seqFL2 = 1;
  }
  if (sign == 65 && seqFL1 == 0) {
    digitalWrite(ballGrip, releasing);
    digitalWrite(ballLiftest, dropLowBall);
    delay(700);
    digitalWrite(ballLifter, dropBall);
    seqFL2 = 1;
    seqFL1 = 1;
  } else if (sign == 56 && seqFL2 == 1) {
    digitalWrite(ballGrip, gripping);
    delay(500);
    digitalWrite(ballLifter, liftBall);
    seqFL2 = 0;
    seqFL1 = 0;

  }
}

void sendingIMU() {
  parsing();
  float imu_yaw_pure = yaw * -1;
  float gyro = yaw;
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
  rollPitch_msg.data[0] = roll;
  rollPitch_msg.data[1] = pitch;
  sendIMU.data[0] = gyro;
}

void kalibrasi() {
  Serial8.begin(115200);
  delay(3000);
  Serial8.write(0xA5);
  Serial8.write(0x54);
  delay(1000);
  Serial8.write(0xA5);
  Serial8.write(0x55);
  delay(100);
}
void parsing() {
  Serial8.write(0XA5);
  Serial8.write(0X51);//send it for each read
  while (Serial8.available()) {
    Re_buf[counter] = (unsigned char)Serial8.read();
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
