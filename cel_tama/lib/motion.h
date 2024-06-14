#ifndef MOTION_H
#define MOTION_H
#include "Point2D.h"
#include <cmath>
#include <vector>

class Kinematic
{
private: 
    static constexpr float anglesM[4] = {45, 135, 225, 315};
    const int R_robot = 0.2585;
    const int R_roda = 0.05;
    float kRoda = 2*M_PI*R_roda;

    float globalAngle;
    int imu_available;

    std::vector<double> enc = {0,0,0,0};
    std::vector<double> Venc = {0,0,0,0};
    std::vector<double> encprev = {0, 0, 0, 0};
public:
    Robot pos;
    void check_imu(float imu_condition){
        if(imu_condition){
            imu_available = 1;
        }else{
            imu_available = 0;;
        }
    }
    float toRad(float degree) {
        return degree * M_PI / 180;
    }
    void forward(Point2D &outFor, float m1,float m2,float m3, float m4);
    void inverse(Robot &motor, float vx, float vy, float vh,float currT);
    void odom(const double orientation);
    void setCurrentPos(double x, double y, double t){
        pos.x = x;
        pos.y = y;
        pos.t = t;
    }
    double angleNormalize(double angle){
        if (angle > M_PI) angle -= 2 * M_PI;
        else if (angle < -M_PI) angle += 2 * M_PI;

        return angle;
    }
};
constexpr float Kinematic::anglesM[4];
void Kinematic::forward(Point2D &outFor, float m1,float m2,float m3, float m4){
    outFor.setX(sin(toRad(anglesM[0]))*m1 + sin(toRad(anglesM[1]))*m2 + sin(toRad(anglesM[2]))*m3 + sin(toRad(anglesM[3]))*m4);
    outFor.setY(cos(toRad(anglesM[0]))*m1 + cos(toRad(anglesM[1]))*m2 + cos(toRad(anglesM[2]))*m3 + cos(toRad(anglesM[3]))*m4);
    outFor.setH((m1 + m2 + m3 + m4) / (4 * R_robot));
}
void Kinematic::inverse(Robot &motor, float vx, float vy, float vh, float currT){  
    globalAngle = imu_available ? currT : 0;
    for (int i = 0; i < 4; ++i) {
        float angle = toRad(anglesM[i] + globalAngle);
        motor.w[i] = (sin(angle) * vx + cos(angle) * vy + vh * R_robot) / R_roda;
    }
}
void Kinematic::odom(const double orientation){
    for(int i = 0; i<4;i++){
        double tick = (enc[i] - encprev[i]);
        Venc[i] = tick * (kRoda / (2 * M_PI));
    }
    Point2D outF;
    forward(outF, Venc[0],Venc[1], Venc[2], Venc[3]);
    Robot velGlobal;

    double angleNorm = angleNormalize(outF.getH());
    double heading = imu_available ? orientation : angleNorm;
    velGlobal.x = (std::cos(heading) * outF.getX()) - (std::sin(heading) * outF.getY());
    velGlobal.y = (std::cos(heading) * outF.getY()) + (std::sin(heading) * outF.getX());

    pos.x += velGlobal.x / 100;
    pos.y += velGlobal.y / 100;
    pos.t = heading;
    encprev = enc;
}
#endif