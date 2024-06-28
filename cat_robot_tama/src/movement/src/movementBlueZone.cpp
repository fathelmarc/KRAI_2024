#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include "pid.h"
#include "motion.h"
#include <std_msgs/Empty.h>
#include "Point2D.h"
#include "control.h"

#include <thread>
float filer1,filer2,filer3,filer4;
void callbackMonitoring(const std_msgs::Float32MultiArray::ConstPtr& arrayMotor){
    filer1 = arrayMotor -> data[0];
    filer2 = arrayMotor -> data[1];
    filer3 = arrayMotor -> data[2];
    filer4 = arrayMotor -> data[3];
}

std::vector<float> data ={0,0,0,0};


float speedChange[4] ={12,20,30,40};
float speedChangeT[4] = {16,24,36,48};

void joystickThread(Joystick* joystick) {
    while (ros::ok()) {
        readJoystickInput(joystick);
        usleep(1000); // Sleep to prevent busy-waiting
    }
}
#define ballrelease 65
#define ballgrip 56
#define launchBall 64
#define offLaunch 46
int main(int argc, char** argv){
    ros::init (argc, argv, "movementBlueZone");
    ros::NodeHandle nh; 
    // ros::Subscriber subMon = nh.subscribe("speedMonitoring", 1000, &callbackMonitoring);
    ros::Subscriber subRP = nh.subscribe("rollPitch", 1000,&callbackRollPitch);
    ros::Publisher pub = nh.advertise<std_msgs::Int32MultiArray>("command", 1000);
    ros::Publisher heartbeat_pub = nh.advertise<std_msgs::Empty>("heartbeat", 10);

    vector<double> ambil1 = computeAverages("/home/m/catR1/src/movement/src/data_cel/ambil1.txt");
    vector<double> tanam1 = computeAverages("/home/m/catR1/src/movement/src/data_cel/tanam1.txt"); 
    vector<double> tanam2 = computeAverages("/home/m/catR1/src/movement/src/data_cel/tanam2.txt");
    vector<double> ambil2 = computeAverages("/home/m/catR1/src/movement/src/data_cel/ambil2.txt");
    vector<double> tanam3 = computeAverages("/home/m/catR1/src/movement/src/data_cel/tanam3.txt");
    vector<double> tanam4 = computeAverages("/home/m/catR1/src/movement/src/data_cel/tanam4.txt");
    vector<double> ambil3 = computeAverages("/home/m/catR1/src/movement/src/data_cel/ambil3.txt");
    vector<double> tanam5 = computeAverages("/home/m/catR1/src/movement/src/data_cel/tanam5.txt");
    vector<double> tanam6 = computeAverages("/home/m/catR1/src/movement/src/data_cel/tanam6.txt");

    Kinematic motion;
    array_msg.data.resize(1);

    pid.parameter(0.5,0,0.0);
    pid.parameterT(0.5,0,0);
    ros::Rate rate(10);
    if (!joy.connected) {
        printf("joy tidak terhubung\n");
        return 1;
    }   
    std::thread joyThread(joystickThread, &joy);
    ROS_INFO("start_robot");
    while(ros::ok()){
        joyControl();
        readJoystickInput(&joy);    


        if(joy.axisStates[L2] > 0){
            array_msg.data[0] = 90;
            lantai2 = 1;   
        }if(joy.axisStates[R2] > 0 ){
            lantai2 = 0;
        }
        if(pitch >= -5){
            array_msg.data[0] = lantai2SetZero;
        }


        int kasus = prevcase;
        // motion.enc = {motion.enc1,motion.enc2,motion.enc3,motion.enc4};
        motion.forward(motion.enc1,motion.enc2,motion.enc3,motion.enc4,motion.yaw);
        // std::vector<float> tamaPose = motion.calcOdometry(motion.yaw);
        // ROS_INFO("\nmonitoring: %d,%d,%d,%d,%d\n,",speed,caseBot,prevcase,lantai2,tanjakkan);
        // ROS_INFO("\nmotorReal: %f, %f, %f, %f\n", motion.enc1,motion.enc2, motion.enc3, motion.enc4);
        // ROS_INFO("\nPosRobot:%f,%f,%f\n", motion.poseX, motion.poseY, motion.poseH);



        if (caseBot == 0 && joy.axisStates[LeftHatX] > 1 && joy.axisStates[LeftHatX] < 32769 ||
            joy.axisStates[LeftHatX] > -32769 && joy.axisStates[LeftHatX] < -1 ||
            joy.axisStates[LeftHatY] > 1 && joy.axisStates[LeftHatY] < 32769 ||
            joy.axisStates[LeftHatY] > -32769 && joy.axisStates[LeftHatY] < -1 ||
            joy.axisStates[RightHatX] > 1 && joy.axisStates[RightHatX] < 32769 ||
            joy.axisStates[RightHatX] < -1 && joy.axisStates[RightHatX] > -32769) {
            float hx = joy.axisStates[LeftHatX] *  speedChange[speed]/1000000;
            float hy = joy.axisStates[LeftHatY] * -speedChange[speed]/1000000;
            float ht = joy.axisStates[RightHatX]*  speedChangeT[speed]/1000000;
            if(lantai2 == 1){
                hx *= -1;
                hy *= -1;
                ht = ht;
            }else{
                hx = hx;
                hy = hy;
                ht = ht;
            }
            motion.check_imu(imuStatus);
            motion.inverse(hx, hy, ht, 0, speedChange[speed]);

            // ROS_INFO("\nmonitoring: %d,%d,%d,%d\n,",speed,caseBot,prevcase,lantai2);
            // ROS_INFO("\nmotorRemote: %f, %f, %f, %f\n", motion.w[0], motion.w[1], motion.w[2], motion.w[3]);
            // ROS_INFO("\nmotorRemote: %f, %f, %f, %f\n", motion.w[0], motion.w[1], motion.w[2], motion.w[3]);
            // ROS_INFO("\nmotorReal: %f, %f, %f, %f\n", filer1, filer2, filer3, filer4);
            // ROS_INFO("\nPosRobot: %f, %f, %f\n", motion.poseX, motion.poseY, motion.poseH);
            // ROS_INFO("\nremote: %f, %f, %f\n",hx,hy,ht);
        }
        
        
        else if(joy.buttonStates[L1]){
            array_msg.data[0] = L1;
        }else if(joy.buttonStates[R1]){
            array_msg.data[0] = R1;
        }



        else if(joy.buttonStates[SQUARE]){
            if(kasus!=0 && lantai2 == 0){
                addData(kasus, motion.poseX, motion.poseY, motion.poseH);
                kasus = 0;
            }
            if(lantai2){
                array_msg.data[0] = 56; //grip
            }else{
                array_msg.data[0] = SQUARE;
            }
            ROS_INFO("\nCase:%d POSE x:%f, y:%f, h:%f\n", prevcase, motion.poseX, motion.poseY, motion.poseH);
        }else if(joy.buttonStates[CIRCLE]){
            if(kasus!=0 && lantai2 == 0){
                addData(kasus, motion.poseX, motion.poseY, motion.poseH);
                kasus = 0;
            }
            if(lantai2){
                array_msg.data[0] = 65; //release
            }else{
                array_msg.data[0] = CIRCLE;
            }
            ROS_INFO("\nCase:%d POSE x:%f, y:%f, h:%f\n", prevcase, motion.poseX, motion.poseY, motion.poseH);
        }
        else if(joy.axisStates[ax_x] > 0){
            if(kasus!=0 && lantai2 == 0){
                addData(kasus, motion.poseX, motion.poseY, motion.poseH);
                kasus = 0;
            }
            array_msg.data[0] = CIRCLE1;
        }
        
        
        else if(joy.buttonStates[share] && lantai2 == 1){
            array_msg.data[0] = launchBall;
        }else if(joy.buttonStates[ps] && lantai2 == 1){
            array_msg.data[0] = offLaunch;
        }



        else if(joy.axisStates[ax_x] < 0 ){
            array_msg.data[0] = SQUARE1;    
        }
            


        else if(joy.buttonStates[share]){
            array_msg.data[0] = 31;
        }else if(joy.buttonStates[option]){
            array_msg.data[0] = 21;
        }
        
        else if(caseBot == 1){
            motion.move_to(0,6,0);
            motion.move_to( ambil1[0] * -1,6,0);

            array_msg.data[0] = 31;
            caseBot = 0;
        }else if(caseBot == 2){ //tanam 1
            motion.move_to(tanam1[0] * -1, tanam1[1],0);
            caseBot = 0;
        }else if(caseBot == 3){ // tanam 2
            caseBot = 0;
        }else if(caseBot == 4){ // ambil 2
            motion.move_to(ambil2[0] * -1  , ambil2[1] + 5,0);
            caseBot = 0;
        }else if(caseBot == 5){
            motion.move_to(tanam3[0] * -1, tanam3[1], 0);
            caseBot = 0;
        }else if(caseBot == 6){
            caseBot = 0;
        }else if(caseBot == 7){
            motion.move_to(ambil3[0] * -1, ambil3[1] + 5,0);
            caseBot = 0;
        }else if(caseBot == 8){
            motion.move_to(tanam5[0] * -1, tanam5[1],0);
            caseBot = 0;
        }else if(caseBot == 9){
            motion.move_to(tanam6[0] * -1, tanam6[1],0);
            caseBot = 0;
        }
    
        else{
            motion.inverse(0,0,0,0,0);
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
