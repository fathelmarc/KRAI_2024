#ifndef PID_H
#define PID_H
#include <chrono>
class PID{
    float prevError;
    double eIntegral;
    double eDerivative;
    double eProportional;
    float u;
    using clock = std::chrono::high_resolution_clock;
    using time_point = std::chrono::time_point<clock>;

    time_point prevTime;
    public:
    double delta();
    float calculatePID(float error, float limit,bool condition);
};
#endif
