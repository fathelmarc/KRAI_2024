#ifndef MOTION_H
#define MOTION_H
#include "fethernet.h"
#include "pidController.h"
#include "control.h"
string str;
// float deltaTime = calculateDeltaTime();
void parsing(){
    atas.terimaData(sockfd);
    str = atas.meg;
    string s;
    stringstream ss(str);
    vector<string> v;
    while (getline(ss, s, ',')) {
        v.push_back(s);
    }
    for (int i = 0; i < v.size(); i++) {
        in[i] = atoi(v[i].c_str());
    }
    stopRight = in[0];
    stopLeft = in[1];
    gyro = in[2];
    // printf("%d,%d,%d\n",stopRight,stopLeft,gyro);
}
float toRad(float degree) {
  return degree * M_PI / 180;
}

//1 = 69 | new 1 = 67
//2 = 68 | new 2 = 66
//3 = 67 | new 3 = 69
//4 = 66 | new 4 = 68
double enc1;
double enc2;
double enc3;
double enc4;
void forKinematic(){
    parsing();
    roda1.terimaData(sockfd);
    roda2.terimaData(sockfd);
    roda3.terimaData(sockfd);
    roda4.terimaData(sockfd);

    enc1 = roda1.en1-roda1.enprev1;
    enc2 = roda2.en2-roda2.enprev2;
    enc3 = roda3.en3-roda3.enprev3;
    enc4 = roda4.en4-roda4.enprev4;
    
    double m1= (enc1*kRoda/PPR)/4;
    double m2= (enc2*kRoda/PPR)/4;
    double m3= (enc3*kRoda/PPR)/4;
    double m4= (enc4*kRoda/PPR)/4;

    roda1.enprev1 = roda1.en1;
    roda2.enprev2 = roda2.en2;
    roda3.enprev3 = roda3.en3;
    roda4.enprev4 = roda4.en4;

    float fx = sin(toRad(angleM1))*m1 + sin(toRad(angleM2))*m2 + sin(toRad(angleM3))*m3 + sin(toRad(angleM4))*m4;
    float fy = cos(toRad(angleM1))*m1 + cos(toRad(angleM2))*m2 + cos(toRad(angleM3))*m3 + cos(toRad(angleM4))*m4;
    passX += fx;
    passY += fy;
    passT = gyro;
    // printf("\nm1 =%f\nm2 =%f\nm3 =%f\nm4 =%f\n",roda1.en1,roda2.en2,roda3.en3,roda4.en4);
}

void inKinematic(float vx, float vy, float vt, float currT){
    parameter(400,40,0);//0.000032,0);
    float v1 = ((sin(toRad(angleM1 + currT)) * vx) + (cos(toRad(angleM1 + currT)) * vy) + (vt * R_robot))/R_roda;
    float v2 = ((sin(toRad(angleM2 + currT)) * vx) + (cos(toRad(angleM2 + currT)) * vy) + (vt * R_robot))/R_roda;
    float v3 = ((sin(toRad(angleM3 + currT)) * vx) + (cos(toRad(angleM3 + currT)) * vy) + (vt * R_robot))/R_roda;
    float v4 = ((sin(toRad(angleM4 + currT)) * vx) + (cos(toRad(angleM4 + currT)) * vy) + (vt * R_robot))/R_roda;
    float motor[4] = {calculatePID(v1-(enc1/PPR/deltaTime) , 150),calculatePID(v2-(enc2/PPR/deltaTime), 150),calculatePID(v3-(enc3/PPR/deltaTime), 150),calculatePID(v4-(enc4/PPR/deltaTime), 150)};
    for (int i = 0 ; i<=4; i++){
        if(motor[i] < 0.05 && motor[i] > -0.05){
            v1=0;v2=0;v3=0;v4=0;
        }
    }
    if(caseBot == 0){
        std::thread thread1(&Device::kirimData, &roda1, sockfd, std::to_string(v1));
        std::thread thread2(&Device::kirimData, &roda2, sockfd, std::to_string(v2));
        std::thread thread3(&Device::kirimData, &roda3, sockfd, std::to_string(v3));
        std::thread thread4(&Device::kirimData, &roda4, sockfd, std::to_string(v4));        
    }
    std::thread thread1(&Device::kirimData, &roda1, sockfd, std::to_string(motor[0]));
    std::thread thread2(&Device::kirimData, &roda2, sockfd, std::to_string(motor[1]));
    std::thread thread3(&Device::kirimData, &roda3, sockfd, std::to_string(motor[2]));
    std::thread thread4(&Device::kirimData, &roda4, sockfd, std::to_string(motor[3]));
    thread1.join();
    thread2.join();
    thread3.join();
    thread4.join();
}
void addData(int kasus) {
    forKinematic();
    ofstream file;
    stringstream filename;
    float prevPassX, prevPassY;
    if (kasus > 0 && kasus <= 12) {
        if (kasus == 1) {
            // printf("add to ambil1\n");
            filename << "dataF1/ambil" << 1 << ".txt";
        }else if(kasus == 2){
            // printf("add to ambil2\n");
            filename << "dataF1/ambil" << 2 << ".txt";
        }
         else if(kasus == 5) {
            // printf("add to ambil3\n");
            filename << "dataF1/ambil" << 3 << ".txt";
        } else if(kasus == 6){
            // printf("add to ambil4\n");
            filename << "dataF1/ambil" << 4 << ".txt";
        } else if(kasus == 9){
            // printf("add to ambil5\n");
            filename << "dataF1/ambil" << 5 << ".txt";
        } else if(kasus == 10){
            // printf("add to ambil6\n");
            filename << "dataF1/ambil" << 6 << ".txt";
        } else if(kasus == 3){
            // printf("add to tanam1\n");
            filename << "dataF1/tanam" << 1 << ".txt";
        } else if(kasus == 4){
            // printf("add to tanam2\n");
            filename << "dataF1/tanam" << 2 << ".txt";
        } else if(kasus == 7){
            // printf("add to tanam3\n");
            filename << "dataF1/tanam" << 3 << ".txt";
        } else if(kasus == 8){
            // printf("add to tanam4\n");
            filename << "dataF1/tanam" << 4 << ".txt";
        } else if(kasus == 11){
            // printf("add to tanam5\n");
            filename << "dataF1/tanam" << 5 << ".txt";
        } else if(kasus == 12){
            // printf("add to tanam6\n");
            filename << "dataF1/tanam" << 6 << ".txt";
        }

        file.open(filename.str(), ios::app);
        if(passX != prevPassX && passY != prevPassY){
            file<<passX<<","<<passY<<","<<passT<<endl;
        }
        prevPassY = passY;
        prevPassX = passX;
        file.close();
    }
}
#endif