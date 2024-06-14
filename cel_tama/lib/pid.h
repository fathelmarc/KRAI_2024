#ifndef PID_H
#define PID_H
#include <chrono>
class PID{
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
#endif