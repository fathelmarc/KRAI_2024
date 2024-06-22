#ifndef PID_H
#define PID_H
#include <chrono>
class PID{
private:
    float kp,ki,kd;
    float prevError;
    double eIntegral;
    double eDerivative;
    double eProportional;
    float u;
    float kpT = 0;
    float kiT = 0;
    float kdT = 0;
    using clock = std::chrono::high_resolution_clock;
    using time_point = std::chrono::time_point<clock>;

    time_point prevTime;
public:
    void parameter(float kp_, float ki_, float kd_);
    void parameterT(float kp_, float ki_, float kd_);
    double delta();
    float calculatePID(float error, float limit,bool condition);
};

#include "pid.h"

void PID::parameter(float kp_, float ki_, float kd_){
    kp = kp_;
    ki = ki_;
    kd = kd_;
}
void PID::parameterT(float kp_, float ki_, float kd_){
    kpT = kp_;
    kiT = ki_;
    kdT = kd_;  
}
double PID::delta() {
    time_point currentTime = clock::now();
    std::chrono::duration<double> delta_time_seconds = currentTime - prevTime;
    auto delta_time_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(delta_time_seconds);
    auto delta_time_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(delta_time_seconds);
    prevTime = currentTime;
    return delta_time_seconds.count();
}
float PID::calculatePID(float error, float limit,bool condition){
    if(condition){
        eProportional = error;
        if (eProportional > 180) {
            eProportional -= 360;
        } else if (eProportional < -180) {
            eProportional += 360;
        }
    }else{
        eProportional = error;
    }
    double deltaT = delta(); 
    eIntegral += eProportional * deltaT;
    eDerivative = (eProportional - prevError)/deltaT;
    prevError = eProportional;
    u = kp*eProportional + ki*eIntegral + kd*eDerivative;
    float uT = kpT * eProportional + kiT * eIntegral + kdT * eDerivative;
    float output = condition ? uT : u;
    return std::fmax(-limit, std::fmin(output, limit));
}

#endif