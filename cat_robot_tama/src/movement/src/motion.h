#ifndef MOTION_H
#define MOTION_H

#include <cmath>
#include <vector>

class Kinematic {
private:
    std::vector<int> anglesM = {45, 135, 225, 315};
    const float R_robot = 0.2585; // Constant radius of the robot
    const float R_roda = 0.05;   // Constant radius of the wheel
    const float kRoda = 2 * M_PI * R_roda; // Constant for calculations involving wheel radius
    const float PPR = 200 * 19.6; // Assuming PPR is constant, made it const

    float globalAngle = 0; // Variable for storing the global angle, initialized to 0
    int imu_available = 0; // Flag indicating if IMU data is available, initialized to 0

    std::vector<float> Venc = {0, 0, 0, 0}; // Motor velocities calculated from encoder values
    std::vector<float> encprev = {0, 0, 0, 0}; // Previous encoder values

public:
    float motor1, motor2, motor3, motor4;
    float poseX, poseY, poseH;

    std::vector<float> enc = {0, 0, 0, 0};
    std::vector<float> w = {0, 0, 0, 0};

    void check_imu(float imu_condition) {
        imu_available = imu_condition ? 1 : 0;
    }
    float toRad(float degree) {
        return degree * M_PI / 180.0;
    }
    void forward(float m1, float m2, float m3, float m4, float heading) {
        enc = {m1, m2, m3, m4};
        for (int i = 0; i < 4; ++i) {
            float tick = enc[i] - encprev[i];
            Venc[i] = (tick * kRoda / PPR) / 4.0;
        }
        float x = sin(toRad(anglesM[0])) * Venc[0] + sin(toRad(anglesM[1])) * Venc[1] +
                sin(toRad(anglesM[2])) * Venc[2] + sin(toRad(anglesM[3])) * Venc[3];

        float y = cos(toRad(anglesM[0])) * Venc[0] + cos(toRad(anglesM[1])) * Venc[1] +
                cos(toRad(anglesM[2])) * Venc[2] + cos(toRad(anglesM[3])) * Venc[3];

        float h = (Venc[0] + Venc[1] + Venc[2] + Venc[3]) / (4.0 * R_robot);

        poseX += x;
        poseY += y;
        poseH = heading;
        encprev = enc;
    }
    void inverse(float vx, float vy, float vh, float currT) {
        globalAngle = imu_available ? currT : 0;
        for (int i = 0; i < 4; ++i) {
            float angle = toRad(anglesM[i] + globalAngle);
            w[i] = (sin(angle) * vx + cos(angle) * vy + vh * R_robot) / R_roda;
            w[i] = fmax(-5, fmin(w[i], 5));
        }
    }
};

#endif // MOTION_H
