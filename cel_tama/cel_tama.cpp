#include "iostream"
#include "chrono"
#include "thread"
#include "fethernet.h"
#include "control.h"
#include "pid.h"
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
void waitMillis(int milliseconds) 
{ 
    clock_t endwait;
    endwait = clock() + milliseconds * (CLOCKS_PER_SEC / 1000); 
    while (clock() < endwait) {
        inKinematic(0,0,0,0);
    } 
}

int main(){
    
    startEthernet();
    // plt::ion(); // Aktifkan mode interaktif matplotlib
    // pid.parameter(0.032,0,0);
    pid.parameter(0.04,0,0.000001);
    pid.parameterT(0.01,0,0);
    vector<double> ambil1 = computeAverages("data_cel/ambil1.txt");
    vector<double> ambil2 = computeAverages("data_cel/ambil2.txt");
    vector<double> ambil3 = computeAverages("data_cel/ambil3.txt");
    vector<double> ambil4 = computeAverages("data_cel/ambil4.txt");
    vector<double> ambil5 = computeAverages("data_cel/ambil5.txt");
    vector<double> ambil6 = computeAverages("data_cel/ambil6.txt");


    vector<double> tanam1 = computeAverages("data_cel/tanam1.txt");
    vector<double> tanam2 = computeAverages("data_cel/tanam2.txt");
    vector<double> tanam3 = computeAverages("data_cel/tanam3.txt");
    vector<double> tanam4 = computeAverages("data_cel/tanam4.txt");
    vector<double> tanam5 = computeAverages("data_cel/tanam5.txt");
    vector<double> tanam6 = computeAverages("data_cel/tanam6.txt");
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
            atas.kirimData(sockfd,std::to_string(hold));
            inKinematic(hx,hy,ht,yaw);
        }

        //ballLifter
        else if(joy.axisStat[ax_x] < 0 && tanjakkan == 2){
            speed = 2;
            for(int i =0; i<3000;i++){
                // inCaseAuto();
                atas.kirimData(sockfd, std::to_string(hornLift));
            }
        }else if(joy.axisStat[ax_x] > 0 && tanjakkan == 2){
            for(int i=0; i<3000;i++){
                // inCaseAuto();
                atas.kirimData(sockfd, std::to_string(hornDown));
            }
        }
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
                // std::cout<<"real:"<<passX<<","<<passY<<","<<passT<<"ambil"<<prevcase<<std::endl;
            }
        }
        else if(joy.bStat[CIRCLE]){
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
            // std::cout<<"real:"<<passX<<","<<passY<<","<<passT<<"tanam"<<prevcase<<std::endl;
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
            for(int i = 0;i<3000;i++){
                readJoystickInput(&joy);
                atas.kirimData(sockfd, std::to_string(L1));
            }
            pid.target(4,ambil1[1] + 4,0);
            for(int i =0; i<3000;i++){
                readJoystickInput(&joy);
                atas.kirimData(sockfd, std::to_string(startGripper));
                inKinematic(0,0,0,yaw);
            }
            pid.target(ambil1[0],ambil1[1] + 4,0);//15.87784035	4.275307269
            std::cout<<ambil1[0]<<","<<ambil1[1]<<"ambil1"<<std::endl;
            caseBot = 0;
        }else if(caseBot == 2){
            pid.target(ambil2[0],ambil2[1]+5,0);//amnbil2 16.7784547019867	3.12796921456953
            for(int i= 0;i<1000;i++){
                readJoystickInput(&joy);
                atas.kirimData(sockfd, std::to_string(R1));
            }
            wait(2);
            pid.targetSlow(ambil2[0],ambil2[1]+3,0);//amnbil2 16.7784547019867	3.12796921456953
            std::cout<<ambil2[0]<<","<<ambil2[1]<<"ambil2"<<std::endl;
            caseBot =0;
        }else if(caseBot == 3){
            for(int i = 0; i<1000;i++){
                readJoystickInput(&joy);
                atas.kirimData(sockfd, std::to_string(R1hard));
            }
            std::cout<<tanam1[0]<<"."<<tanam1[1]<<"tanam1"<<std::endl;
            pid.target(tanam1[0],tanam1[1],0);//tanam pertama
            // waitMillis(300);
            caseBot = 0;
        }else if(caseBot == 4){
            for(int i =0; i<1000;i++){
                readJoystickInput(&joy);
                atas.kirimData(sockfd, std::to_string(L1hard));
            }
            std::cout<<tanam2[0]<<","<<tanam2[1]<<"tanam2"<<std::endl;
            pid.target(tanam2[0],tanam2[1],0);//tanam kedua
            // waitMillis(300);
            caseBot = 0;
        }

        else if(caseBot == 5){
            for(int i = 0; i<1000;i++){
                readJoystickInput(&joy);
                atas.kirimData(sockfd, std::to_string(L1));
            }
            std::cout<<ambil3[0]<<","<<ambil3[1]<<"ambil3"<<std::endl;
            pid.target(ambil3[0],8.136726923,0);//ambil ke3

            caseBot = 0;
        }else if(caseBot == 6){
            pid.target(ambil4[0],10.5217,0);//ambil ke 4
            for(int i = 0;i<1000;i++){
                readJoystickInput(&joy);
                atas.kirimData(sockfd, std::to_string(R1));
            }
            std::cout<<ambil4[0]<<","<<ambil4[1]<<"ambil4"<<std::endl;
            wait(2);
            pid.targetSlow(ambil4[0],ambil4[1]+2,0);
            caseBot = 0;
        }else if(caseBot == 7){
            for(int i = 0;i<3000;i++){
                readJoystickInput(&joy);
                atas.kirimData(sockfd, std::to_string(R1hard));
            }
            pid.target(tanam3[0],tanam3[1],0);//tanam ke3
            std::cout<<tanam3[0]<<","<<tanam3[1]<<"tanam3"<<std::endl;
            // waitMillis(300);
            caseBot =0;
        }else if(caseBot == 8){
            for(int i = 0;i<1000;i++){
                readJoystickInput(&joy);
                atas.kirimData(sockfd, std::to_string(L1hard));
            }
            pid.target(tanam4[0],tanam4[1],0);//tanam  k4
            std::cout<<tanam4[0]<<","<<tanam4[1]<<"tanam4"<<std::endl;
            // waitMillis(300);
            caseBot =0;
        }

        else if(caseBot == 9){
            for(int i =0;i<1000;i++){
                readJoystickInput(&joy);
                atas.kirimData(sockfd, std::to_string(L1));
            }
            pid.target(ambil5[0],ambil5[1]+2,0);//ambil ke 5
            std::cout<<ambil5[0]<<","<<ambil5[1]<<"ambil5"<<std::endl;
            caseBot =0;
            
        }else if(caseBot == 10){
            pid.target(ambil6[0],ambil6[1]+5,0);//ambil ke 6
            for(int i = 0; i<1000;i++){
                readJoystickInput(&joy);
                atas.kirimData(sockfd, std::to_string(R1));
            }
            wait(2);
            pid.targetSlow(ambil6[0],ambil6[1]+2, 0);
            std::cout<<ambil6[0]<<","<<ambil6[1]<<"ambil6"<<std::endl;
            caseBot = 0;
        }else if(caseBot == 11){
            for(int i = 0; i<1000;i++){
                readJoystickInput(&joy);
                atas.kirimData(sockfd, std::to_string(R1hard));
            }
            pid.target(tanam5[0],tanam5[1],0);//tanam ke 5

            std::cout<<tanam5[0]<<","<<tanam5[1]<<"tanam5"<<std::endl;
            // waitMillis(300);
            caseBot =0;
        }else if(caseBot == 12){
            for(int i =0;i<1000;i++){
                readJoystickInput(&joy);
                atas.kirimData(sockfd, std::to_string(L1hard));
            }
            pid.target(tanam6[0],tanam6[1],0);//tanam ke 6
            std::cout<<tanam6[0]<<","<<tanam6[1]<<"tanam6"<<std::endl;
            // waitMillis(300);
            caseBot =0;
        }else if(caseBot == 13){
            pid.target(44.26194271, tanam6[1] - 9, 0);
            pid.target(0,tanam6[1] - 8,0);
            for(int i = 0;i<1000;i++){
                atas.kirimData(sockfd,std::to_string(R1));
            }
            speed = 3;
            caseBot = 0;
        }
            
        else{
            // std::cout<<"get into else"<<std::endl;
            atas.kirimData(sockfd, std::to_string(hold)); 
            inKinematic(0,0,0,yaw);
        }
        fflush(stdout);
        usleep(16000);
    }
    close(sockfd);
    return 0;   
}
