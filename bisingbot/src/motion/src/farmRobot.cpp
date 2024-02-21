#include <ros/ros.h>
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/UInt16.h"

#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"
#include "iostream"
#include "chrono"
#include "thread"
#include "ethernet.h"

#include <vector>
#include <cstdlib>
#include <ctime>
using namespace std;

#define R_robot 0.2585
#define R_roda 0.05
#define Droda  0.1
#define PPR 200
#define MAXSPEED 0.3
//max robot gerak = 0.58 m/s atau 2 km/jam

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

double ky,kx,kt;

float kRoda = M_PI*Droda;
volatile float prevErr_x,prevErr_y,prevErr_t;
volatile long prevTime;

float lx,ly,lt,XFilt,YFilt,TFilt,
	  deltaTime,passX,passY,errT,errX,errY,
	  v1,v2,v3,v4,uPrevT,uPrevX,uPrevY,x,y,t;
volatile float errprevX,
	  errprevY, errprevT;
int passT, in[50],sens;
string cmd;
string str;
//1 = 69 | new 1 = 67
//2 = 68 | new 2 = 66
//3 = 67 | new 3 = 69
//4 = 66 | new 4 = 68
Device atas("192.168.0.70",5555);
Device roda1("192.168.0.67",5555);
Device roda2("192.168.0.66",5555);
Device roda3("192.168.0.69",5555);
Device roda4("192.168.0.68",5555);
int Xtar,Ytar,Xcurr,Ycurr;
void time(){
	ros::Time current_time = ros::Time::now();
	uint32_t currTime = current_time.nsec;
	deltaTime = currTime - prevTime;
	prevTime = clock();
}

void readKey(const std_msgs::String::ConstPtr& order){
	cmd =order->data.c_str();
}
void readCam(const geometry_msgs::Quaternion::ConstPtr& see){
	Xtar = see->x;
	Xcurr = see->y;
	Ytar = see->z;
	Ycurr = see->w;
}

double kalman(double U);
double kalman(double U){
	//kalman function def
	static const double R = 10;
	static const double H = 1.00;
	static double Q = 20;
	static double P = 0;
	static double U_hat = 0; // result
	static double K =0.3;

	K = P*H/(H*P*H+R);
	U_hat = U_hat + K*(U-H*U_hat);

	P = (1-K*H)*P+Q;
	return U_hat;
}

void ForKinematic(){
	roda1.terimaData(sockfd);
	roda2.terimaData(sockfd);
	roda3.terimaData(sockfd);
	roda4.terimaData(sockfd);
	double m1= (((roda1.en1-roda1.enprev1)*kRoda/PPR)/4);
	double m2= (((roda2.en2-roda2.enprev2)*kRoda/PPR)/4);
	double m3= (((roda3.en3-roda3.enprev3)*kRoda/PPR)/4);
	double m4= (((roda4.en4-roda4.enprev4)*kRoda/PPR)/4);
	roda1.enprev1 = roda1.en1;
	roda2.enprev2 = roda2.en2;
	roda3.enprev3 = roda3.en3;
	roda4.enprev4 = roda4.en4;

	float fx = sin(toRad(angleM1))*m1 + sin(toRad(angleM2))*m2 + sin(toRad(angleM3))*m3 + sin(toRad(angleM4))*m4;
	float fy = cos(toRad(angleM1))*m1 + cos(toRad(angleM2))*m2 + cos(toRad(angleM3))*m3 + cos(toRad(angleM4))*m4;
	passX += fx;
    passY += fy;
    // ROS_INFO("%f,%f,%f,%f\n",roda1.en1,roda2.en2,roda3.en3,roda4.en4);
}
void InKinematic(float vx, float vy, float vt){
	v1 = (sin(toRad(angleM1)) * vx + cos(toRad(angleM1)) * vy + vt * R_robot)/R_roda;
	v2 = (sin(toRad(angleM2)) * vx + cos(toRad(angleM2)) * vy + vt * R_robot)/R_roda;
	v3 = (sin(toRad(angleM3)) * vx + cos(toRad(angleM3)) * vy + vt * R_robot)/R_roda;
	v4 = (sin(toRad(angleM4)) * vx + cos(toRad(angleM4)) * vy + vt * R_robot)/R_roda;
	// roda1.kirimData(sockfd, std::to_string(v1));
	// roda2.kirimData(sockfd, std::to_string(v2));
	// roda3.kirimData(sockfd, std::to_string(v3));
	// roda4.kirimData(sockfd, std::to_string(v4));
	// ROS_INFO("\nv1=%f\nv2=%f\nv3=%f\nv4=%f\n",v1,v2,v3,v4);
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

float kp =0.4;//mulus(0.13);//3.06;//1.08;//1.8;//0.51;//0.85;//0.36
float ki =0;//.03;//.013;//mulus(0);//.017;//0.0372;//0;//0.102;//0;
float kd =0.00396;//.00186;//mulus(0);//.01317;//6.83;//0;//0.6375;//0;

float kpt =0.04;//0.06;
float kit =0.005;//0.0062;
float kdt =0.02;//0.0016;
void PIDx(float targetX){
	// ForKinematic();
	time();
	errX = targetX - passX;

	float errInX =+ errX * deltaTime;

	float errDerX = (errX - errprevX)/deltaTime;
	errprevX = errX;

	float ux = kp * errX + ki * errInX + kd * errDerX;
	lx = fmax(-MAXSPEED,fmin(ux,MAXSPEED));
}
void PIDy(float targetY){
	// ForKinematic();
	time();
	errY = targetY - passY;
	float errInY = errInY + errY * deltaTime;

	float errDerY = (errY - errprevY)/deltaTime;
	errprevY = errY;

	float uy = kp * errY + ki * errInY + kd * errDerY;
	ly = fmax(-MAXSPEED,fmin(uy,MAXSPEED));
}
void PIDt(float targetT){
	// ForKinematic();
	// parsing();
	time();
	passT = in[0];
	errT = targetT - passT;
	if (errT > 180 || errT < -180){
		errT = in[0] - targetT;
	}

	float errInT = errInT + errT * deltaTime;
	
	float errDerT = (errT - errprevT)/deltaTime;
	errprevT = errT;
	
	float ut = kpt * errT + kit * errInT + kdt * errDerT;
	lt = fmax(-MAXSPEED,fmin(ut,MAXSPEED));
}

void Adjust(float x, float y, float t){
	PIDt(t);
	InKinematic(x,y,lt);
}
void setPos(float targetX, float targetY, float targetT){
	// std::thread t(PIDt,targetT);
	// std::thread y(PIDy,targetY);
	// std::thread x(PIDx,targetX);
	// t.join();
	// y.join();
	// x.join();
	PIDt(targetT);
	PIDx(targetX);
	PIDy(targetY);
    InKinematic(lx, ly, lt);
	// ROS_INFO("%f,%f,%f",passY,ky);
    // ROS_INFO("%f    %f",errInX,errInY);
	ROS_INFO("X%f Y%f T%d\nv1=%f\nv2=%f\nv3=%f\nv4=%f\n",passX,passY,in[0],v1,v2,v3,v4);
    // ROS_INFO("%f,%f,%f",lx,ly,lt);
}
int main(int argc, char **argv){
	// mkIkat();
    ros::init (argc, argv, "farm");
    ros::NodeHandle nh;
    ros::Subscriber tombol = nh.subscribe("sign",1000,readKey);
	ros::Subscriber mata = nh.subscribe("chat_vis",1000,readCam);
    ros::Rate rate(2000);
	ROS_INFO("start robot");
    while(ros::ok()){
		ROS_INFO("%d,%d,%d,%d\n",Xtar,Xcurr,Ytar,Ycurr);
		// parsing();
		// ROS_INFO("%d\n",in[0]);
		// if(cmd == "start"){
		// 	setPos(10,3.5,0);
		// 	passX = x;
		// 	passY = y;
		// 	passT = t;
		// }else if(cmd == "q"){
		// 	setPos(x,y+20,0);
		// }else if(cmd == "w"){
		// 	setPos(x+3,y-20,0);
		// }else if(cmd == "e"){
		// 	setPos(x,y+20,0);
		// }

		// else if(cmd == "maju"){
		// 	setPos(0,30,0);
		// }else if(cmd == "mundur"){
		// 	setPos(0,-30,0);	
		// }else if(cmd == "kanan"){
		// 	setPos(30,0,0);

		// }else if(cmd == "kiri"){
		// 	setPos(-30,0,0);
		// }
		// else if(cmd == "pause"){
		// 	InKinematic(0,0,0);
		// 	// roda1.kirimData(sockfd, std::to_string(0));
		// 	// roda2.kirimData(sockfd, std::to_string(0));
		// 	// roda3.kirimData(sockfd, std::to_string(0));
		// 	// roda4.kirimData(sockfd, std::to_string(0));
		// }else if(cmd == "Balik"){ //b
		// 	setPos(0,0,0);
		// }else if(cmd == "putar kanan"){
		// 	setPos(0,0,180);
		// }else if(cmd == "putar kiri"){
		// 	Adjust(0,0,-180);
		// }else if(cmd == "mati"){
        //     ros::shutdown();
        // }

		// else {
		// 	InKinematic(0,0,0);
		// }
		ros::spinOnce();
        rate.sleep();
    }
	close(sockfd);
    return 0;
}
