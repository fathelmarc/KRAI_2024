#ifndef MOTION_H
#define MOTION_H

#include "pid.h"

#include <cmath>
#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include <vector>

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <linux/input.h>

int roll, pitch;
void callbackRollPitch(const std_msgs::Int32MultiArray::ConstPtr& rollPitch_msg){
    roll = rollPitch_msg -> data[0];
    pitch = rollPitch_msg -> data[1];
}
std_msgs::Int32MultiArray rollPitch;



std_msgs::Int32MultiArray array_msg;

PID pid;

#define LeftHatX 0
#define LeftHatY 1
#define RightHatX 3
#define RightHatY 4
const int SILANG = 0;
const int CIRCLE = 1;
const int CIRCLE1 = -1;
const int SQUARE1 = -3;
const int TRIANGLE =2;
const int SQUARE =3;
const int L1 =4;
const int R1 =5;
const int L2 =2;
const int R2 =5;
const int ax_y =7; //axis < 0
// const int down 7 //axis >0
const int ax_x  =6; //axis >0
// const int right 6 //axis <0
const int triangle =2;//button
const int circle =1;//button
const int square =3;//button
// const int ex 0 //button
const int ps =10; //button
const int option =9;//button
const int share =8; //button


struct Joystick
{
    bool connected;
    char buttonCount;
    float* buttonStates;
    char axisCount;
    float* axisStates;
    char name[128];
    int file;
};
Joystick openJoystick(const char* fileName)
{
	Joystick j = {0};
	int file = open(fileName, O_RDONLY | O_NONBLOCK);
	if (file != -1)
	{
		ioctl(file, JSIOCGBUTTONS, &j.buttonCount);
		j.buttonStates = (float*)calloc(j.buttonCount, sizeof(float));
		ioctl(file, JSIOCGAXES, &j.axisCount);
		j.axisStates = (float*)calloc(j.axisCount, sizeof(float));
		ioctl(file, JSIOCGNAME(sizeof(j.name)), j.name);
		j.file = file;
		j.connected = true;
	}
	return j;
}

void readJoystickInput(Joystick* joystick)
{
    while(1){
        js_event event;
        int bytesRead = read(joystick->file, &event, sizeof(event));
        if (bytesRead == 0 || bytesRead == -1) return;

        if (event.type == JS_EVENT_BUTTON && event.number < joystick->buttonCount) {
            joystick->buttonStates[event.number] = event.value ? 1:0;
        }
        if (event.type == JS_EVENT_AXIS && event.number < joystick->axisCount) {
            joystick->axisStates[event.number] = event.value;
        }
    }
}

static int prevTanjakkan = 0;
static int tanjakkan = 0;
static int afterTanjakkan = 0;
const char* fileName = "/dev/input/js0";
Joystick joy = openJoystick(fileName);
static int speed;
int prevcase,caseBot,imuStatus;
int lantai2;
const int lantai2SetZero = 90;
void joyControl(){
    readJoystickInput(&joy);
    //kecepatan
    static int axisCont = 0;
    static int prevAxisValue = 0;
    static int send_gripper_ball = 0;
     if(pitch <= -5 && afterTanjakkan == 0 && prevcase >= 12){
        tanjakkan = prevTanjakkan + 1;
        prevTanjakkan = tanjakkan;

        afterTanjakkan = 1;
    }else if(pitch >= -1 && afterTanjakkan == 1){
        tanjakkan = prevTanjakkan + 1;
        prevTanjakkan = tanjakkan;
        afterTanjakkan = 2;
    }
    if (joy.axisStates[ax_y] > prevAxisValue && !axisCont) {
        speed--;
        axisCont =1;
    }
    if (joy.axisStates[ax_y] < prevAxisValue && !axisCont) {
        speed++;
        axisCont = 1;
    }else if(joy.axisStates[ax_y] == 0){
        axisCont = 0;
    }else if(speed >= 3){
        speed = 3;
    }else if(speed <= -1){
        speed = 0;
    }
    prevAxisValue = joy.axisStates[ax_y];
    static int silangPressed = 0;
    static int trianglePressed = 0;
    if (joy.buttonStates[SILANG] && !silangPressed) {
        if(prevcase < 10) {
            caseBot = prevcase + 1;
            prevcase = caseBot;
        }
        silangPressed = 1;
    }
    
    if (!joy.buttonStates[SILANG]) {
        silangPressed = 0;
    }
    if(joy.buttonStates[TRIANGLE] && !trianglePressed){
        if(prevcase > 1){
            caseBot = prevcase - 1;
            prevcase = caseBot;
        }
        trianglePressed =1;
    }
    if(!joy.buttonStates[TRIANGLE]){
        trianglePressed =0;
    }
    if (prevcase > 10) {
        caseBot = 9;
        prevcase = caseBot;
    }
    if(joy.buttonStates[ps]){
        caseBot = 0;
    }if(joy.buttonStates[option]){
        prevcase = 0;
    }
}

class Kinematic {
private:
    std::vector<int> anglesM = {45, 135, 225, 315};
    const float R_robot = 0.2585; // Constant radius of the robot
    const float R_roda = 0.05;   // Constant radius of the wheel
    const float kRoda = 2 * M_PI * R_roda; // Constant for calculations involving wheel radius
    const float PPR = 200 * 19.6; // Assuming PPR is constant, made it const

    float globalAngle = 0; // Variable for storing the global angle, initialized to 0
    int imu_available = 0; // Flag indicating if IMU data is available, initialized to 0

    ros::NodeHandle nh;
    std::vector<float> Venc = {0, 0, 0, 0}; // Motor velocities calculated from encoder values
    std::vector<float> encprev = {0, 0, 0, 0}; // Previous encoder values

public:
    ros::Publisher pubSpeed;
    ros::Subscriber sub;
    ros::Subscriber subIMU;
    std_msgs::Float32MultiArray pesanPWM;
    Kinematic(){
        pubSpeed = nh.advertise<std_msgs::Float32MultiArray>("speed_motor",10);
        sub = nh.subscribe("encoder", 1000, &Kinematic::callbackArrayMotor,this);
        subIMU = nh.subscribe("IMU", 1000, &Kinematic::callBackIMU,this);
    };
    float enc1 = 0,enc2 = 0,
          enc3 = 0,enc4 = 0;
    void callbackArrayMotor(const std_msgs::Int32MultiArray::ConstPtr& arrayMotor){
        enc1 = static_cast<float>(arrayMotor -> data[0]);
        enc2 = static_cast<float>(arrayMotor -> data[1]);
        enc3 = static_cast<float>(arrayMotor -> data[2]);
        enc4 = static_cast<float>(arrayMotor -> data[3]);
    }

    int yaw = 0,yawPure = 0;
    void callBackIMU(const std_msgs::Int32MultiArray::ConstPtr& pesanImu){
        yaw = static_cast<float>(pesanImu -> data[0]);
        yawPure = static_cast<float>(pesanImu -> data[1]);
    }


    float motor1 = 0, motor2 = 0, motor3 = 0, motor4 = 0;
    float poseX = 0, poseY = 0, poseH = 0;

    std::vector<float> enc = {0, 0, 0, 0};
    std::vector<float> w = {0, 0, 0, 0};

    void check_imu(float imu_condition) {
        imu_available = imu_condition ? 1 : 0;
    }
    float toRad(float degree) {
        return degree * M_PI / 180.0;
    }
    void forward(float m1, float m2, float m3, float m4, float heading) {
        enc = {enc1, enc2, enc3, enc4};
        for (int i = 0; i < 4; ++i) {
            float tick = enc[i] - encprev[i];
            Venc[i] = (tick * kRoda / PPR);
        }
        float x = sin(toRad(anglesM[0])) * Venc[0] + sin(toRad(anglesM[1])) * Venc[1] +
                sin(toRad(anglesM[2])) * Venc[2] + sin(toRad(anglesM[3])) * Venc[3];

        float y = cos(toRad(anglesM[0])) * Venc[0] + cos(toRad(anglesM[1])) * Venc[1] +
                cos(toRad(anglesM[2])) * Venc[2] + cos(toRad(anglesM[3])) * Venc[3];

        float h = (Venc[0] + Venc[1] + Venc[2] + Venc[3]) / (4.0 * R_robot);

        poseX += (x * -1);
        poseY += (y * -1);
        poseH = heading;
        encprev = enc;
    }
    void inverse(float vx, float vy, float vh, float currT, float speed) {
        globalAngle = imu_available ? currT : 0;
        for (int i = 0; i <= 4; ++i) {
            float angle = toRad(anglesM[i] + currT);
            w[i] = (sin(angle) * vx + cos(angle) * vy + vh * R_robot) / R_roda;
            w[i] = fmax(-speed, fmin(w[i], speed));
        }
        pesanPWM.data = {w[0],w[1],w[2],w[3]};
        pubSpeed.publish(pesanPWM);
    }
    void move_to(float targetX, float targetY, float targetH) {
        ros::Rate   rate(10); // Loop at 10 Hz
        while (true) {
            // Process incoming messages
            ros::spinOnce();

            joyControl();
            forward(enc1, enc2, enc3, enc4, yaw);

            float errorX = targetX - poseX;
            float errorY = targetY - poseY;
            float errorH = targetH - yaw;

            float distance = sqrt(pow(errorX, 2) + pow(errorY, 2));
            float headingError = errorH;

            if (fabs(distance) <= 1 || caseBot == 0) {
                break; // Exit loop if close enough to target
            }

            float getH = atan2(errorX, errorY);

            float controlXY = pid.calculatePID(distance, 1.7, 0);
            float controlH = pid.calculatePID(headingError, 0.6, 1);
            float velX = controlXY * sin(getH);
            float velY = controlXY * cos(getH);
            float velH = controlH;

            inverse(velX, velY, velH, yaw, 23);
            pesanPWM.data = { w[0], w[1], w[2], w[3] };
            pubSpeed.publish(pesanPWM);
            
            // ROS_INFO("\nmonitoring: %d,%d,%d,%d\n,",speed,caseBot,prevcase,lantai2);
            // ROS_INFO("\nSendSpeed: %f, %f, %f, %f\n\nPoseRobot: %f, %f, %d\n",
                    //   w[0], w[1], w[2], w[3],poseX, poseY, yaw);
            rate.sleep();
        }
    }
};

#endif // MOTION_H
