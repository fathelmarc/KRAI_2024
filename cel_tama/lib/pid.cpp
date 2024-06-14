#include "pid.h"
struct param{
    float kp,ki,kd;
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
    float output = u;
    return std::fmax(-limit, std::fmin(output, limit));
}
