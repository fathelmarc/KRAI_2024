#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "iostream"
using namespace std;

float PPR = 200;

float px =0;
float py =0;
float pt =0;

float en1prev =0;
float en2prev =0;
float en3prev =0;
float en4prev =0;

float v1 = 0;
float v2 = 0;
float v3 = 0;
float v4 = 0;

#define kpx 1
#define kpy 1
#define kpt 1

#define R_roda 5
#define R_robot 16.149

float en1 = 0;
float en2 = 0;
float en3 = 0;
float en4 = 0;
float gy =0;


void ambil_enc(const geometry_msgs::Quaternion::ConstPtr& enc){
    en1 = enc->x;
    en2 = enc->y;
    en3 = enc->z;
    en4 = enc->w;
}
void ambil_imu(const geometry_msgs::Quaternion::ConstPtr& imu){
    gy = imu->x;
}

float toRad(float degree) {
  return degree * M_PI / 180;
}

void ForKinematic(){
    float m1=(en1 - en1prev) * 2 * M_PI / (PPR);
    float m2=(en2 - en2prev) * 2 * M_PI / (PPR);
    float m3=(en3 - en3prev) * 2 * M_PI / (PPR);
    float m4=(en4 - en4prev) * 2 * M_PI / (PPR);

    en1prev = en1;
    en2prev = en2;
    en3prev = en3;
    en4prev = en4;

        
    float var = sqrt(2);
    float fx = ((var * m1) + (var * m2) + (-var * m3) + (-var * m4)) / 4;
    float fy = ((var * m1) + (-var * m2) + (-var * m3) + (var * m4)) / 4;

    px += fx;
    py += fy;

}

void InKinematic(float vx, float vy, float vt){
    v1 = sin(toRad(45)) * vx + cos(toRad(45)) * vy + R_robot * vt/R_roda;
    v2 = sin(toRad(135)) * vx + cos(toRad(135)) * vy + R_robot * vt/R_roda;
    v3 = sin(toRad(225)) * vx + cos(toRad(225)) * vy + R_robot * vt/R_roda;
    v4 = sin(toRad(315)) * vx + cos(toRad(315)) * vy + R_robot * vt/R_roda;
}

void PID(float x, float y, float t){
    ForKinematic();

    float ex = x - px;
    float ux = kpx * ex;
    ux = fmaxf(-3.7,fminf(ux,3.7));

    float ey = y - py;
    float uy = kpy * ey;
    uy = fmaxf(-3.7,fminf(uy,3.7));

    float et = t - gy;
    if (et> 180 || et<180){
        et = gy - t;
    }
    float ut = kpt * et;
    ut = fmaxf(-3.7,fminf(ut,3.7));

    InKinematic(ux, uy, ut);
    ROS_INFO("imu: %f, motor: %f", gy, v1);
}



int main(int argc, char **argv){
    ros::init (argc, argv, "autoTanam");
    ros::NodeHandle nh;
    //while(ros::ok()){
    
    ros::Subscriber enc = nh.subscribe("chat_enc",1000,ambil_enc);
    ros::Subscriber imu = nh.subscribe("chat_imu",1000,ambil_imu);
    ros::Publisher sendAllMotor = nh.advertise<geometry_msgs::Quaternion>("odomet", 1000);

    while(ros::ok()){
        ros::spinOnce();
        PID(0,0,90);
        
        geometry_msgs::Quaternion sendMotor;

        sendMotor.x = v1;
        sendMotor.y = v2;
        sendMotor.z = v3;
        sendMotor.w = v4;
        
        // Publish the Quaternion message
        sendAllMotor.publish(sendMotor);
        
    }
    return 0;
}
