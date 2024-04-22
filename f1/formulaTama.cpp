#include "iostream"
#include "chrono"
#include "thread"
#include "fethernet.h"
#include "control.h"
#include "wind.h"

float kp =0.016;//0.0465;//0.4;//mulus(0.13);//3.06;//1.08;//1.8;//0.51;//0.85;//0.36
float ki =0;//0.00003;//.03;//0;//.03;//.013;//mulus(0);//.017;//0.0372;//0;//0.102;//0;
float kd =0.0000000001;//0.003;//.00396;//0.00396;//.00186;//mulus(0);//.01317;//6.83;//0;//0.6375;//0;

float kpt =0.013;//0.0123;//0.06;
float kit =0;//0.000000000001;//.005;//0.0062;
float kdt =0;//0;//.02;//0.0016;

double deltaTime = calculateDeltaTime();
float pidT(float targetT,float currT){
    forKinematic();
    float errprevT;
    float errT = targetT - currT;
    // printf("o= %f\n",currT);
    if(errT < -180 ){
        // float selisih = 180 - currT;
        // currT = (selisih + 180)*-1;
        // printf("i= %f\n",currT);
        errT =currT - targetT;
    }
    float  errInT;
    errInT += errT * deltaTime;
    
    float errDerT = (errT - errprevT)/deltaTime;
    errprevT = errT;
    
    float ut = kpt * errT + kit * errInT + kdt * errDerT;
    float lt = fmax(-MAXSPEEDT,fmin(ut,MAXSPEEDT));
    return lt ;
    // printf("e= %f\n",errT);
}
void adjust (float targetX,float targetY, float targetT, int adHead){
    forKinematic();
    float velT = pidT(targetT,passT);
    inKinematic(targetX,targetY,velT,adHead);
}
float kp_,ki_,kd_;
void parameter(float p_, float i_, float d_){
    kp_ = p_;
    ki_ = i_;
    kd_ = d_;
}
float calculatePID(float error, float limit){
    parameter(0.037,0.000032,0.0000000000001);//0.000032,0);
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
void go_to(float TargetX,float TargetY,float TargetH)
{   
    while(1){
        inCaseAuto();
        displayPosition(window,trail,passX, passY);
        float errorX = TargetX-passX;
        float errorY = TargetY-passY;
        float errorH = TargetH-passT;

        float errorXY = sqrt(errorX*errorX+errorY*errorY);

        float errorT = atan2(errorX,errorY);

        float kontrol_sbXY = calculatePID(errorXY,2);
        float kontrolT = calculatePID(errorH, 2);

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
void just_go(float TargetX,float TargetY,float TargetH)
{
    inCaseAuto();
    displayPosition(window,trail,passX, passY);
    float errorX = TargetX-passX;
    float errorY = TargetY-passY;
    float errorH = TargetH-passT;

    float errorXY = sqrt(errorX*errorX+errorY*errorY);

    float errorT = atan2(errorX,errorY);

    float kontrol_sbXY = calculatePID(errorXY,2);
    float kontrolT = calculatePID(errorH, 1);

    float velocityX = kontrol_sbXY*sin(errorT);
    float velocityY = kontrol_sbXY*cos(errorT);
    float velocityH = kontrolT;
    inKinematic(velocityX,velocityY,velocityH,passT);
    // printf("%f\n", velocityY);
}
int main(){
    startEthernet();
    // plt::ion(); // Aktifkan mode interaktif matplotlib
    if (!joy.connected) {
        printf("joy tidak terhubung\n");
        return 1;
    }
    printf("start robot\n");
    while(window.isOpen()){
        std::thread eventThread(check, std::ref(window));
        eventThread.join();
        inCaseAuto();
        displayPosition(window,trail,passX, passY);
        // printf("%f,%f,%d\n",passX,passY,passT);
        int kasus = prevcase;
        // showPosition(passX, passY);
        if(caseBot ==0 && joy.axisStat[LeftHatX] > 1 && joy.axisStat[LeftHatX] < 32769 ||joy.axisStat[LeftHatX] > -32769 && joy.axisStat[LeftHatX] < -1 ||joy.axisStat[LeftHatY] > 1 && joy.axisStat[LeftHatY] < 32769 ||joy.axisStat[LeftHatY] > -32769 && joy.axisStat[LeftHatY] < -1 ||joy.axisStat[RightHatX] > 1 && joy.axisStat[RightHatX] < 32769 ||joy.axisStat[RightHatX] < -1 && joy.axisStat[RightHatX] > -32769){
            float hx = joy.axisStat[LeftHatX] *  speedChange[speed]/1000000;
            float hy = joy.axisStat[LeftHatY] * -speedChange[speed]/1000000;
            float ht = joy.axisStat[RightHatX]*  speedChangeT[speed]/1000000;
            if (moveImu == 0){
                inKinematic(hx, hy, ht,passT);}
            else if(moveImu == 1){
                adjust(hx, hy, 0, passT);
            }
        }
        
        //gripper

        else if(joy.bStat[SQUARE]){//petak
            if(kasus!=0){
                addData(kasus);
                kasus = 0;
            }
            for (int i = 0;i<3000;i++){
                // printf("petak");
                atas.kirimData(sockfd, std::to_string(SQUARE));
            }
        }else if(joy.bStat[CIRCLE]){

            if(kasus!=0){
                addData(kasus);
                kasus = 0;
            }
            for (int i = 0;i<3000;i++){
                // printf("bulat");
                atas.kirimData(sockfd, std::to_string(CIRCLE));
			}
        }
        //gripper
        else if(joy.axisStat[6] < 0){//petak

            if(kasus!=0){
                addData(kasus);
                kasus = 0;
            }
            for (int i = 0;i<3000;i++){
                // printf("petak");
                atas.kirimData(sockfd, std::to_string(SQUARE1));
            }
        }else if(joy.axisStat[6] > 0){

            if(kasus!=0){
                addData(kasus);
                kasus = 0;
            }
            for (int i = 0;i<3000;i++){
                // printf("bulat");
                atas.kirimData(sockfd, std::to_string(CIRCLE1));
			}
        }
        //putar
        else if(joy.bStat[L1]){//l1
            for(int i =0; i<1000; i++){
                atas.kirimData(sockfd, std::to_string(L1));
            }
        }		
		else if(joy.bStat[R1]){//r1
            for (int i = 0;i<1000;i++){
                atas.kirimData(sockfd, std::to_string(R1));
            }
        }
        else if(joy.bStat[8]){
            just_go(0,0,0);
        }

        //position
        else if(caseBot == 1){
            for(int i =0; i<1000;i++){
                atas.kirimData(sockfd, std::to_string(L1));
                atas.kirimData(sockfd, std::to_string(90));
            }
            go_to(0,6,0);
            go_to(14.4615434782609,6.06739,0);
            // go_to(0,100,0);
            // setPos(0,15+5,0);
            // setPos(15+5,15,0);
            // setPos(15,10+5,0);
            // setPos(0+5,0+5,0);
            // setPos(1.5,5.0,0);
            // go_to(0,20,0);//ambil pertama 14.47	3.09
            // go_to(20, 20,0);
            // go_to(20,0,0);
            // go_to(0,0,0);
            // go_to(0, 3+5,0);
            // go_to(15+5,3,0);

            caseBot = 0;
        }else if(caseBot == 2){
            go_to(18.2958588235294,6.17013,0);//amnbil2 16.7784547019867	3.12796921456953
            for(int i= 0;i<1000;i++){
                atas.kirimData(sockfd, std::to_string(R1));
            }
            caseBot =0;
        }else if(caseBot == 3){
            go_to(20.9084764705882,26.8165882352941,0);//tanam pertama
            for(int i = 0; i<1000;i++){
                atas.kirimData(sockfd, std::to_string(R1));
            }
            caseBot = 0;
        }else if(caseBot == 4){
            go_to(19.9105733333333,20.63714,0);//tanam kedua
            for(int i =0; i<1000;i++){
                atas.kirimData(sockfd, std::to_string(L1));
            }
            caseBot = 0;
        }

        else if(caseBot == 5){
            for(int i = 0; i<1000;i++){
                atas.kirimData(sockfd, std::to_string(L1));
            }
            go_to(28.08299375,6.38343,0);//ambil ke3
            caseBot = 0;
        }else if(caseBot == 6){
            go_to(30.4399285714286,6.59225,1);//ambil ke 4
            for(int i = 0;i<1000;i++){
                atas.kirimData(sockfd, std::to_string(R1));
            }caseBot = 0;
        }else if(caseBot == 7){
            atas.kirimData(sockfd, std::to_string(R1));
            go_to(33.2912826086957,27.373547826087,0);//tanam ke3
            caseBot =0;
        }else if(caseBot == 8){
            go_to(32.22004375,21.34726875,0);//tanam  k4
            for(int i = 0;i<1000;i++){
                atas.kirimData(sockfd, std::to_string(L1));
            }
            caseBot =0;
        }

        else if(caseBot == 9){
            for(int i =0;i<1000;i++){
                atas.kirimData(sockfd, std::to_string(L1));
            }
            go_to(40.5068875,6.05465,1);//ambil ke 5
            caseBot =0;
            
        }else if(caseBot == 10){
            go_to(43.2599142857143,6.60778,0);//ambil ke 6
            for(int i = 0; i<1000;i++){
                atas.kirimData(sockfd, std::to_string(R1));
            }
            caseBot = 0;
        }else if(caseBot == 11){
            for(int i = 0; i<1000;i++){
                atas.kirimData(sockfd, std::to_string(R1));
            }go_to(47.0347375,28.8019791666667,2);//tanam ke 5
            caseBot =0;
        }else if(caseBot == 12){
            go_to(45.4556058823529,22.1591823529412,0);//tanam ke 6
            for(int i =0;i<1000;i++){
                atas.kirimData(sockfd, std::to_string(L1));
            }caseBot =0;
        }

        else{
            atas.kirimData(sockfd,std::to_string(hold));
            inKinematic(0,0,0,passT);
        }
        fflush(stdout);
        usleep(16000);
    }
    close(sockfd);
    return 0;   
}