#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
#include <chrono>
#include <math.h>

double calculateDeltaTime() {
    static auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> delta = end - start;
    return delta.count(); // Mengembalikan delta time dalam detik
}

float deltaTime = calculateDeltaTime();
float kp_,ki_,kd_;
void parameter(float p_, float i_, float d_){
    kp_ = p_;
    ki_ = i_;
    kd_ = d_;
}
float calculatePID(float error, float limit){
    float prevError;
    float eProportional = error;
    float eIntegral;
    eIntegral += error * deltaTime;
    float eDerivative = (error*prevError)/deltaTime;
    prevError = error;
    float u = kp_*eProportional + ki_*eIntegral + kd_*eDerivative;
    float limSpeed = fmax(-limit,fmin(u, limit));
    return limSpeed;
}
#endif