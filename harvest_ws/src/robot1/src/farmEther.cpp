#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/UInt16.h"

#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "iostream"
#include "chrono"
#include "thread"
#include "ethernet.h"
using namespace std;

#define R_robot 0.2585
#define R_roda 0.05
#define Droda  0.1
#define PPR 200
#define MAXSPEED 0.5

float gripper = 1;
float conveyor = 2;
float piston = 3;
float stopGripper = 11;
float stopConveyor = 22;
float stopPiston = 33;

float openServo = 4;
float closeServo = 44;

const int angleM1 = 45;
const int angleM2 = 135;
const int angleM3 = 225;
const int angleM4 = 315;

float kRoda = M_PI*Droda;
volatile float prevErr_x,prevErr_y,prevErr_t;
volatile long prevTime;

float lx,ly,lt,XFilt,YFilt,TFilt,
	  deltaTime,passX,passY,errprevX,
	  errprevY, errprevT,v1,v2,v3,
	  v4,x,y,t,errT,errX,errY,gy;
int passT, in[50],sens;
int hit =0;
string press,cmd;
string str;
//1 = 69
//2 = 68
//3 = 67
//4 = 66
Device atas("192.168.0.70",5555);
Device roda1("192.168.0.67",5555);
Device roda2("192.168.0.66",5555);
Device roda3("192.168.0.69",5555);
Device roda4("192.168.0.68",5555);

void time(){
	ros::Time current_time = ros::Time::now();
	uint32_t currTime = current_time.nsec;
	deltaTime = currTime - prevTime;
	prevTime = clock();
}

void readKey(const std_msgs::String::ConstPtr& order){
	cmd =order->data.c_str();
	// ROS_INFO("Key %s",order->data.c_str());
}

void ForKinematic(){
	roda1.terimaData(sockfd);
	roda2.terimaData(sockfd);
	roda3.terimaData(sockfd);
	roda4.terimaData(sockfd);
	double m1= (((roda1.en1/*-roda1.enprev1*/)*kRoda/PPR)/4);
	double m2= (((roda2.en2/*-roda2.enprev2*/)*kRoda/PPR)/4);
	double m3= (((roda3.en3/*-roda3.enprev3*/)*kRoda/PPR)/4);
	double m4= (((roda4.en4/*-roda4.enprev4*/)*kRoda/PPR)/4);
	float var = sqrt(2);
	// float fx = ((var * m1) + (var * m2) + (-var * m3) + (-var * m4))/4;
	// float fy = ((var * m1) + (-var * m2) + (-var * m3) + (var * m4))/4;
	float fx = sin(toRad(angleM1))*m1 + sin(toRad(angleM2))*m2 + sin(toRad(angleM3))*m3 + sin(toRad(angleM4))*m4;
	float fy = cos(toRad(angleM1))*m1 + cos(toRad(angleM2))*m2 + cos(toRad(angleM3))*m3 + cos(toRad(angleM4))*m4;
	passX += fx;
    passY += fy;
	roda1.enprev1 = roda1.en1;
	roda2.enprev2 = roda2.en2;
	roda3.enprev3 = roda3.en3;
	roda4.enprev4 = roda4.en4;
	ROS_INFO("\nroda1=%f\nroda2=%f\nroda3=%f\nroda4=%f\n",roda1.en1,roda2.en2,roda3.en3,roda4.en4);
}
void InKinematic(float vx, float vy, float vt){
	v1 = (sin(toRad(angleM1)) * vx + cos(toRad(angleM1)) * vy + vt * R_robot)/R_roda;
	v2 = (sin(toRad(angleM2)) * vx + cos(toRad(angleM2)) * vy + vt * R_robot)/R_roda;
	v3 = (sin(toRad(angleM3)) * vx + cos(toRad(angleM3)) * vy + vt * R_robot)/R_roda;
	v4 = (sin(toRad(angleM4)) * vx + cos(toRad(angleM4)) * vy + vt * R_robot)/R_roda;
	roda1.kirimData(sockfd, std::to_string(v1));
	roda2.kirimData(sockfd, std::to_string(v2));
	roda3.kirimData(sockfd, std::to_string(v3));
	roda4.kirimData(sockfd, std::to_string(v4));
	// ROS_INFO("\nR1= %f\nR2= %f\nR3= %f\nR4= %f\n",v1,v2,v3,v4);
}

void parsing(){
	atas.terimaData(sockfd);
	str = atas.data;
	string s;
    stringstream ss(str);
    vector<string> v;
    while (getline(ss, s, ',')) {
        v.push_back(s);
    }
    for (int i = 0; i < v.size(); i++) {
        in[i] = atoi(v[i].c_str());
    }
	sens = in[1];
}

float kp  =1.5;//0.36
float ki  =0;
float kd  =0;
float kpt =0.005;
float kit =0;
float kdt =0;

void PIDx(float target){
	ForKinematic();
	time();
	float uPrevX;
	errX = target - passX;
	float errInX = errInX + errX * deltaTime;
	float errDerX = (errX - errprevX)/deltaTime;
	errprevX = errX;
	float ux = kp * errX + ki * errInX + kd * errDerX;
	lx = fmax(-MAXSPEED,fmin(ux,MAXSPEED));
	XFilt = 0.854*XFilt + 0.0728 * lx + 0.0728 * uPrevX;
	uPrevX = lx;
}
void PIDy(float target){
	ForKinematic();
	time();
	float uPrevY;
	errY = target - passY;
	float errInY = errInY + errY * deltaTime;
	float errDerY = (errY - errprevY)/deltaTime;
	errprevY = errY;
	float uy = kp * errY + ki * errInY + kd* errDerY;
	ly = fmax(-MAXSPEED,fmin(uy,MAXSPEED));
	YFilt = 0.854*YFilt + 0.0728 * ly + 0.0728 * uPrevY;
	uPrevY = ly;
}
void PIDt(float target){
	ForKinematic();
	parsing();
	time();
	float uPrevT;
	passT = in[0];
	errT = target - passT;
	if (errT > 180 || errT < -180){
		errT = passT - target;
	}
	float errInT = errInT + errT * deltaTime;
	float errDerT = (errT - errprevT)/deltaTime;
	errprevT = errT;
	float ut = kpt * errT + kit * errInT + kdt * errDerT;
	lt = fmax(-MAXSPEED,fmin(ut,MAXSPEED));
	TFilt = 0.854*TFilt + 0.0728 * lt + 0.0728 * uPrevT;
	uPrevT = lt;
}
void setPos(float targetX, float targetY, float targetT){
	PIDx(targetX);
	PIDy(targetY);
	PIDt(targetT);
    InKinematic(lx, ly, lt);
	x = passX;
	y = passY;
	t = passT;
	// ROS_INFO("%f,%f,%d\n",passX,passY,passT);
}
int main(int argc, char **argv){
	mkIkat();
    ros::init (argc, argv, "farmEther");
    ros::NodeHandle nh;
    ros::Subscriber tombol = nh.subscribe("sign",1000,readKey);
    ros::Rate rate(2000);
	ROS_INFO("start robot");
    while(ros::ok()){
		parsing();
		// ROS_INFO("%d\n",sens);
		if(cmd == "start"){
			if(sens == 0){
				roda1.kirimData(sockfd, std::to_string(0));
				roda2.kirimData(sockfd, std::to_string(0));
				roda3.kirimData(sockfd, std::to_string(0));
				roda4.kirimData(sockfd, std::to_string(0));
				atas.kirimData(sockfd, std::to_string(gripper));
				atas.kirimData(sockfd, std::to_string(closeServo));
				usleep(2000);
				atas.kirimData(sockfd, std::to_string(stopGripper));
			}
			else if(sens == 1){
				setPos(-30,-3,0);
				atas.kirimData(sockfd, std::to_string(openServo));
			}
		}else if(cmd == "maju"){
			setPos(0,10,0);
		}else if(cmd == "mundur"){
			setPos(0,-10,0);	
		}else if(cmd == "kanan"){
			setPos(10,0,0);
			while(sens == 0){
				InKinematic(0,0,0);
			}
		}else if(cmd == "kiri"){
			setPos(-10,0,0);
		}
		else if(cmd == "pause"){
			InKinematic(0,0,0);
		}else if(cmd == "Balik"){ //b
			setPos(0,0,0);
		}else if(cmd == "putar kiri"){
			setPos(0,0,-4);
		}else if(cmd == "putar kanan"){
			setPos(0,0,4);
		}

		else {
			InKinematic(0,0,0);
		}
		ros::spinOnce();
        rate.sleep();
    }
	close(sockfd);
    return 0;
}
