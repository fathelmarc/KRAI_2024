#ifndef PID_H
#define PID_H
#include <iostream>
#include "fethernet.h"
#include "wind.h"

double deltaTime = calculateDeltaTime();
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
    void parameter(float kp_, float ki_, float kd_){
        kp = kp_;
        ki = ki_;
        kd = kd_;
    }
    void parameterT(float kp_, float ki_, float kd_){
        kpT = kp_;
        kiT = ki_;
        kdT = kd_;
    }
    double delta() {
        time_point currentTime = clock::now();
        std::chrono::duration<double> delta_time_seconds = currentTime - prevTime;
        auto delta_time_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(delta_time_seconds);
        auto delta_time_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(delta_time_seconds);
        prevTime = currentTime;
        // return delta_time_seconds.count();
        return delta_time_seconds.count();
    }
    float calculatePID(float error, float limit,bool condition){
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
        // std::cout<<deltaT<<std::endl;
        return std::fmax(-limit, std::fmin(output, limit));
        
    }
    void target(float TargetX,float TargetY,float TargetH)
    {   
        while(1){
            
            //parameter(0.032,0.000035,0.0000000000005);//0.000032,0);
            inCaseAuto();
            displayPosition(window,trail,passX, passY);
            float errorX = TargetX-passX;
            float errorY = TargetY-passY;
            float errorH = TargetH-passT;
            // if(errorH < -180){
            //     errorH = passT - TargetH;
            // }
            float errorXY = sqrt(errorX*errorX+errorY*errorY);

            float errorT = atan2(errorX,errorY);

            float kontrol_sbXY = calculatePID(errorXY,2,0);
            float kontrolT = calculatePID(errorH, 1,1);

            float velocityX = kontrol_sbXY*sin(errorT);
            float velocityY = kontrol_sbXY*cos(errorT);
            float velocityH = kontrolT;
            inKinematic(velocityX,velocityY,velocityH,passT);
            // printf("%f,%f,%f\n", velocityX,velocityY,velocityH);
            if(errorX < 1 && errorY < 1 && errorX > -1 && errorY > -1 || caseBot == 0){
                break;
            }
        }
    }
    void targetSlow(float TargetX,float TargetY,float TargetH)
    {   
        while(1){
            inCaseAuto();
            //parameter(0.032,0.000035,0.0000000000005);//0.000032,0);
            // atas.kirimData(sockfd,std::to_string(hold));
            displayPosition(window,trail,passX, passY);
            float errorX = TargetX-passX;
            float errorY = TargetY-passY;
            float errorH = TargetH-passT;

            float errorXY = sqrt(errorX*errorX+errorY*errorY);

            float errorT = atan2(errorX,errorY);

            float kontrol_sbXY = calculatePID(errorXY,1,0);
            float kontrolT = calculatePID(errorH, 1,1);

            float velocityX = kontrol_sbXY*sin(errorT);
            float velocityY = kontrol_sbXY*cos(errorT);
            float velocityH = kontrolT;
            inKinematic(velocityX,velocityY,velocityH,passT);
            // printf("%f\n", velocityY);
            if(errorX < 1 && errorY < 1 && errorX > -1 && errorY > -1 || caseBot == 0){
                break;
            }
        }
    }
    void targetNoWhile(float TargetX,float TargetY,float TargetH)
    {
        inCaseAuto();
        displayPosition(window,trail,passX, passY);
        float errorX = TargetX-passX;
        float errorY = TargetY-passY;
        float errorH = TargetH-passT;

        float errorXY = sqrt(errorX*errorX+errorY*errorY);

        float errorT = atan2(errorX,errorY);

        float kontrol_sbXY = calculatePID(errorXY,2,0);
        float kontrolT = calculatePID(errorH, 1,1);

        float velocityX = kontrol_sbXY*sin(errorT);
        float velocityY = kontrol_sbXY*cos(errorT);
        float velocityH = kontrolT;
        inKinematic(velocityX,velocityY,velocityH,passT);
        // printf("%f\n", velocityY);
    }
};


class  Robot
{
private:
    /* data */
public:
     Robot(/* args */);
    ~ Robot();
};

 Robot:: Robot(/* args */)
{
}

 Robot::~ Robot()
{
}

#endif
