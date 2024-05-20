#include "iostream"
#include "chrono"
#include "thread"
#include "fethernet.h"
#include "control.h"
// #include "wind.h"
#include "pid.h"
float kp =0.016;//0.0465;//0.4;//mulus(0.13);//3.06;//1.08;//1.8;//0.51;//0.85;//0.36
float ki =0;//0.00003;//.03;//0;//.03;//.013;//mulus(0);//.017;//0.0372;//0;//0.102;//0;
float kd =0.0000000001;//0.003;//.00396;//0.00396;//.00186;//mulus(0);//.01317;//6.83;//0;//0.6375;//0;

float kpt =0.013;//0.0123;//0.06;
float kit =0;//0.000000000001;//.005;//0.0062;
float kdt =0;//0;//.02;//0.0016;

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

PID pid;
void adjust (float targetX,float targetY, float targetT, int adHead){
    forKinematic();
    float err = targetT - yaw;
    if (err > 180) {
        err -= 360;
    } else if (err < -180) {
        err += 360;
    }
    float velT = pid.calculatePID(err,0.3,1);
    inKinematic(targetX,targetY,velT,adHead);
}
 

void wait (int seconds) 
{ 
  clock_t endwait;
  endwait = clock () + seconds * CLOCKS_PER_SEC ; 
  while (clock() < endwait) {
    inKinematic(0,0,0,0);
  } 
}

int main(){
    startEthernet();
    // plt::ion(); // Aktifkan mode interaktif matplotlib
    // pid.parameter(0.032,0,0);
    pid.parameter(0.03,0,0.000001);
    pid.parameterT(0.01,0,0);
    if (!joy.connected) {
        printf("joy tidak terhubung\n");
        return 1;
    }
    printf("start robot\n");
    while(1){
        // pid.parameter(0.032,0.000035,0.0000000000000);
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
            if(pitch <-2 && pitch >= -5 && tanjakkan != 2){
                adjust(hx,hy,45,yaw);
            }else{
                inKinematic(hx,hy,ht,yaw);
            }
        }

        //ballLifter
        else if(joy.axisStat[ax_x] < 0 && tanjakkan == 2){
            for(int i =0; i<3000;i++){
                atas.kirimData(sockfd, std::to_string(hornLift));
            }
        }else if(joy.axisStat[ax_x] > 0 && tanjakkan == 2){
            for(int i=0; i<3000;i++){
                atas.kirimData(sockfd, std::to_string(hornDown));
            }
        }
        // else if(joy.bStat[SQUARE] && tanjakkan == 2){
        //     send(gripperClose, 1000);
        //     send(lifterUp, 1000);
        //     send(hornLift, 1000);
        //     waitmillis(200);
        //     send(gripperOpen, 1000);

        // }else if(joy.bStat[CIRCLE] && tanjakkan == 2){
        //     send()
        // }
        //gripper
        else if(joy.bStat[SQUARE]){//petak
            if(kasus!=0 && tanjakkan == 0){
                addData(kasus);
                kasus = 0;
            }
            for (int i = 0;i<3000;i++){
                // printf("petak");
                if(prevcase % 2 ==0 || tanjakkan == 2){
                    atas.kirimData(sockfd, std::to_string(SQUARE));
                    // printf("tutup0\n");
                }else {
                    atas.kirimData(sockfd, std::to_string(SQUARE1));
                    // printf("tutup1\n");
                }
            }
        }else if(joy.bStat[CIRCLE]){
            if(kasus!=0 && tanjakkan == 0){
                addData(kasus);
                kasus = 0;
            }
            for (int i = 0;i<3000;i++){
                // printf("bulat");if(prevcase % 2 ==0){
                if(prevcase % 2 == 0 || tanjakkan == 2){
                    atas.kirimData(sockfd, std::to_string(CIRCLE));
                    // printf("buka0\n");
                }else {
                    atas.kirimData(sockfd, std::to_string(CIRCLE1));
                    // printf("buka1\n");
                }
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
        //case 13 titik y = 14 +-
        //case 13 titik x = 0
        // sambil 45
        //case 14 (naik) titik y = 45
        //case 14 (naik) titk x = -9
        //position
        else if(caseBot == 1){
            for(int i =0; i<1000;i++){
                atas.kirimData(sockfd, std::to_string(L1));
                atas.kirimData(sockfd, std::to_string(startGripper));
            }
            pid.target(0,10,0);
            pid.target(10,10,0);
            pid.target(10,0,45);
            pid.target(0,0,45);
            // pid.target(4,6,0);
            // pid.target(14.52793676,6,0);//15.87784035	4.275307269
            caseBot = 0;
        }else if(caseBot == 2){
            pid.target(16.37163611,8.0204,0);//amnbil2 16.7784547019867	3.12796921456953
            for(int i= 0;i<1000;i++){
                atas.kirimData(sockfd, std::to_string(R1));
            }
            wait(2);
            pid.targetSlow(17.1196269,3.883657143,0);//amnbil2 16.7784547019867	3.12796921456953
            
            caseBot =0;
        }else if(caseBot == 3){
            pid.target(17.70369229,23.36150975,0);//tanam pertama
            for(int i = 0; i<1000;i++){
                atas.kirimData(sockfd, std::to_string(R1));
            }
            caseBot = 0;
        }else if(caseBot == 4){
            pid.target(17.54996531,18.26238712,0);//tanam kedua
            for(int i =0; i<1000;i++){
                atas.kirimData(sockfd, std::to_string(L1));
            }
            caseBot = 0;
        }

        else if(caseBot == 5){
            for(int i = 0; i<1000;i++){
                atas.kirimData(sockfd, std::to_string(L1));
            }
            pid.target(26.60280122,8.136726923,0);//ambil ke3
            caseBot = 0;
        }else if(caseBot == 6){
            pid.target(29.28384203,8.5217,0);//ambil ke 4
            for(int i = 0;i<1000;i++){
                atas.kirimData(sockfd, std::to_string(R1));
            }
            wait(2);
            pid.targetSlow(30.07513875,4.824895714,0);
            caseBot = 0;
        }else if(caseBot == 7){
            atas.kirimData(sockfd, std::to_string(R1));
            pid.target(30.54377183,25.6289525,0);//tanam ke3
            caseBot =0;
        }else if(caseBot == 8){
            pid.target(30.52891759,19.82531695,0);//tanam  k4
            for(int i = 0;i<1000;i++){
                atas.kirimData(sockfd, std::to_string(L1));
            }
            caseBot =0;
        }

        else if(caseBot == 9){
            for(int i =0;i<1000;i++){
                atas.kirimData(sockfd, std::to_string(L1));
            }
            pid.target(39.78618428,10.259113636,0);//ambil ke 5
            caseBot =0;
            
        }else if(caseBot == 10){
            pid.target(42.92404417,10.71562,0);//ambil ke 6
            for(int i = 0; i<1000;i++){
                atas.kirimData(sockfd, std::to_string(R1));
            }
            wait(2);
            pid.targetSlow(44.97815822,6.729197857, 0);
            caseBot = 0;
        }else if(caseBot == 11){
            for(int i = 0; i<1000;i++){
                atas.kirimData(sockfd, std::to_string(R1));
            }pid.target(44.32172496,27.22039783,0);//tanam ke 5
            caseBot =0;
        }else if(caseBot == 12){
            pid.target(44.26194271,22.226692,0);//tanam ke 6
            for(int i =0;i<1000;i++){
                atas.kirimData(sockfd, std::to_string(L1));
            }caseBot =0;
        }else if(caseBot == 13){
            pid.target(44.26194271, 8, 0);
            pid.target(0,8,0);
            caseBot = 0;
        }

        else{
            atas.kirimData(sockfd, std::to_string(hold)); 
            inKinematic(0,0,0,passT);
        }
        fflush(stdout);
        usleep(16000);
    }
    close(sockfd);
    return 0;   
}
