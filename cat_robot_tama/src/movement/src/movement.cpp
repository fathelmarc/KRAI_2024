#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include "pid.h"
#include "motion.h"
#include "Point2D.h"
#include "control.h"

#include <thread>
PID pid;
Kinematic motion;
std_msgs::Float32MultiArray pesanPWM;
float filer1,filer2,filer3,filer4;
void callbackMonitoring(const std_msgs::Float32MultiArray::ConstPtr& arrayMotor){
    filer1 = arrayMotor -> data[0];
    filer2 = arrayMotor -> data[1];
    filer3 = arrayMotor -> data[2];
    filer4 = arrayMotor -> data[3];
}

std::vector<float> data ={0,0,0,0};
float errorX;
float errorY;

void move_to(float targetX, float targetY , float targetH){
    motion.forward(enc1, enc2,enc3,enc4,yaw);
    errorX = targetX - motion.poseX;
    errorY = targetY - motion.poseY;
    float errorH = targetH - 0;

    float getXY = (pow(errorX, 2) + pow(errorY, 2));
    float getH = atan2(errorX, errorY);

    float controlXY = pid.calculatePID(getXY, 120, 0);
    float controlH = pid.calculatePID(errorH, 100, 1);
    float velX = controlXY * sin(getH);
    float velY = controlXY * cos(getH);
    float velH = controlH;
    motion.inverse(velX , velY, velH, 0);
    pesanPWM.data = {motion.w[0], motion.w[1], motion.w[2], motion.w[3]};
}

float speedChange[4] = {1.5,2,3,7};
float speedChangeT[4] = {2,2,3,7};
std_msgs::Int32MultiArray array_msg;

void joystickThread(Joystick* joystick) {
    while (ros::ok()) {
        readJoystickInput(joystick);
        usleep(1000); // Sleep to prevent busy-waiting
    }
}
int main(int argc, char** argv){
    if (!joy.connected) {
        printf("joy tidak terhubung\n");
        return 1;
    }   
    ros::init (argc, argv, "movement");

    ros::NodeHandle nh; 
    ros::Subscriber subMon = nh.subscribe("speedMonitoring", 1000, &callbackMonitoring);
    ros::Subscriber sub = nh.subscribe("array_enc", 1000, &callbackArrayMotor);
    ros::Subscriber subIMU = nh.subscribe("IMU", 1000,&callBackIMU);
    ros::Publisher pubSpeed = nh.advertise<std_msgs::Float32MultiArray>("speed_motor",10);
    ros::Publisher pub = nh.advertise<std_msgs::Int32MultiArray>("array_topic", 1000);
    array_msg.data.resize(1);


    std::thread joyThread(joystickThread, &joy);
    ros::Rate rate(10);
    ROS_INFO("start_robot");
    while(ros::ok()){
        joyControl();
        readJoystickInput(&joy);
        // motion.check_imu()
        // motion.enc = {enc1,enc2,enc3,enc4};
        motion.forward(enc1,enc2,enc3,enc4,yaw);
        // std::vector<float> tamaPose = motion.calcOdometry(yaw);
        ROS_INFO("\nPosRobot:%f,%f,%f\n", motion.poseX, motion.poseY, motion.poseH);

        ROS_INFO("\nmotorRemote: %f, %f, %f, %f\n", motion.w[0], motion.w[1], motion.w[2], motion.w[3]);
        ROS_INFO("\nmotorReal: %f, %f, %f, %f\n", filer1, filer2, filer3, filer4);
        if (std::abs(joy.axisStates[LeftHatX]) > 1 || std::abs(joy.axisStates[LeftHatY]) > 1 || std::abs(joy.axisStates[RightHatX]) > 1) {
            float hx = joy.axisStates[LeftHatX] * speedChange[speed] / 10000;
            float hy = joy.axisStates[LeftHatY] * -speedChange[speed] / 10000;
            float ht = joy.axisStates[RightHatX] * speedChangeT[speed] / 10000;
            joyControl();
            motion.inverse(hx, hy, ht, 0);
            pesanPWM.data = {motion.w[0], motion.w[1], motion.w[2], motion.w[3]};
            pubSpeed.publish(pesanPWM);

            ROS_INFO("\nmotorRemote: %f, %f, %f, %f\n", motion.w[0], motion.w[1], motion.w[2], motion.w[3]);
            ROS_INFO("\nmotorReal: %f, %f, %f, %f\n", filer1, filer2, filer3, filer4);
            ROS_INFO("\nPosRobot: %f, %f, %f\n", motion.poseX, motion.poseY, motion.poseH);
            ROS_INFO("\nremote: %f, %f, %f\n",hx,hy,ht);\
        }else if(joy.buttonStates[L1]){
            array_msg.data[0] = L1;
        }else if(joy.buttonStates[R1]){
            array_msg.data[0] = R1;
            
        }else if(joy.buttonStates[SQUARE]){
            array_msg.data[0] = SQUARE;
            
        }else if(joy.buttonStates[CIRCLE]){
            array_msg.data[0] = CIRCLE;
            
        }else if(joy.axisStates[ax_x] < 0 ){
            array_msg.data[0] = SQUARE1;
            
        }
        else if(joy.axisStates[ax_x] > 0){
            array_msg.data[0] = CIRCLE1;
            
        }else if(joy.buttonStates[share]){
            array_msg.data[0] = 31;
        }else if(joy.buttonStates[option]){
            array_msg.data[0] = 21;
        }
        else{
            motion.inverse(0,0,0,0);
            pesanPWM.data = {motion.motor1,motion.motor2,motion.motor3,motion.motor4};
            pubSpeed.publish(pesanPWM);
        }

        pub.publish(array_msg);
        ros::spinOnce();
        rate.sleep();
        fflush(stdout);
        usleep(16000);
    }
    //2 = 1
    //3 = 2
    //4 = 3
    //1 = 4
    joyThread.join();
    return 0;
}
