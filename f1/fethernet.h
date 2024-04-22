
#ifndef FETHERNET_H
#define FETHERNET_H

#include <bits/stdc++.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <termios.h>
#include <unistd.h>

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <linux/input.h>

#include <vector>
#include <cstdlib>
#include <ctime>

using namespace std;

#define R_robot 0.2585
#define R_roda 0.05
#define Droda  0.1
#define PPR 200
//max robot gerak = 0.58 m/s atau 2 km/jam

#define hold 169

const int angleM1 = 45;
const int angleM2 = 135;
const int angleM3 = 225;
const int angleM4 = 315;

float kRoda = M_PI*Droda;

float passY,passX;
int passT;
int gyro,stopRight,stopLeft;
int in[50],sens,i;

#define PORT 5555
#define MAXLINE 1024
char buffer[MAXLINE];
int sockfd,sizeReceive;
struct sockaddr_in serv_addr,cli_addr;
socklen_t len;
struct ps{
char buffer[50];
    int counter;
    float heading;
}ps;
class Device {
public:
    struct sockaddr_in address, client;
    double en1,en2,en3,en4, enprev1,enprev2,enprev3,enprev4;
    double en[4],enprev[4];
    int receive_some,sonic;
    std::string data,meg;
    char buffer[MAXLINE];
    char terima[MAXLINE];
    socklen_t clasLen, clientLen;
    std::string addIP, deviceIP;

    Device(const char *IP, unsigned long colok) {
        memset(&address, 0, sizeof(address));
        memset(&client, 0, sizeof(client));
        deviceIP = IP;
        address.sin_family = AF_INET;
        inet_pton(AF_INET, IP, &address.sin_addr.s_addr);
        address.sin_port = htons(colok);  // Use the provided port parameter
    }
    void kirimData(int soket, std::string msg) {
        sendto(soket, msg.c_str(), strlen(msg.c_str()), MSG_WAITALL, (struct sockaddr *)&address, sizeof(address));
    }

    void terimaData(int socket) {
        receive_some = recvfrom(socket, (char *)buffer, MAXLINE,
        0, (struct sockaddr *)&client,
        &clasLen);
        buffer[receive_some] = '\0';
        addIP = inet_ntoa(client.sin_addr);
        strncpy(terima, buffer, MAXLINE);
        if (addIP == "192.168.0.70") {
            meg = terima;
        }else if(addIP == "192.168.0.67"){
            en1 = atof(terima);
        }else if(addIP == "192.168.0.66"){
            en2 = atof(terima);
        }else if(addIP == "192.168.0.69"){
            en3 = atof(terima);
        }else if(addIP == "192.168.0.68"){
            en4 = atof(terima);
        }else if(addIP == "192.168.0.2"){
            sonic = atoi(terima);
        }
    }
};

string str;
Device atas("192.168.0.70",8888);
Device roda1("192.168.0.67",5555);
Device roda2("192.168.0.66",5555);
Device roda3("192.168.0.69",5555);
Device roda4("192.168.0.68",5555);

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
void forKinematic(){
    parsing();
    roda1.terimaData(sockfd);
    roda2.terimaData(sockfd);
    roda3.terimaData(sockfd);
    roda4.terimaData(sockfd);
    double m1= ((roda1.en1-roda1.enprev1)*kRoda/PPR)/4;
    double m2= ((roda2.en2-roda2.enprev2)*kRoda/PPR)/4;
    double m3= ((roda3.en3-roda3.enprev3)*kRoda/PPR)/4;
    double m4= ((roda4.en4-roda4.enprev4)*kRoda/PPR)/4;

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
double calculateDeltaTime() {
    static auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> delta = end - start;
    return delta.count(); // Mengembalikan delta time dalam detik
}

void inKinematic(float vx, float vy, float vt, float currT){
    float v1 = ((sin(toRad(angleM1 + currT)) * vx) + (cos(toRad(angleM1 + currT)) * vy) + (vt * R_robot))/R_roda;
    float v2 = ((sin(toRad(angleM2 + currT)) * vx) + (cos(toRad(angleM2 + currT)) * vy) + (vt * R_robot))/R_roda;
    float v3 = ((sin(toRad(angleM3 + currT)) * vx) + (cos(toRad(angleM3 + currT)) * vy) + (vt * R_robot))/R_roda;
    float v4 = ((sin(toRad(angleM4 + currT)) * vx) + (cos(toRad(angleM4 + currT)) * vy) + (vt * R_robot))/R_roda;
    std::thread thread1(&Device::kirimData, &roda1, sockfd, std::to_string(v1));
    std::thread thread2(&Device::kirimData, &roda2, sockfd, std::to_string(v2));
    std::thread thread3(&Device::kirimData, &roda3, sockfd, std::to_string(v3));
    std::thread thread4(&Device::kirimData, &roda4, sockfd, std::to_string(v4));
    thread1.join();
    thread2.join();
    thread3.join();
    thread4.join();
}
double kalman(double U);
double kalman(double U){
    //kalman function def
    static const double R = 10;
    static const double H = 1.00;
    static double Q = 20;
    static double P = 0;
    static double U_hat = 0; // result
    static double K =0.3;

    K = P*H/(H*P*H+R);
    U_hat = U_hat + K*(U-H*U_hat);

    P = (1-K*H)*P+Q;
    return U_hat;
}


void startEthernet(){
// buatkan socket untuk deskripsi file
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }
    memset(&serv_addr, 0, sizeof(serv_addr));
    // info server semacam addressnya
    serv_addr.sin_family = AF_INET; //IPv4
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(PORT);
    //ikat socket dengan servernya
    if ( bind(sockfd, (const struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0 ){
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
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
#endif // ETHERNET_H