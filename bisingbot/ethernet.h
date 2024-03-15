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
#define MAXSPEED 0.2
#define MAXSPEEDT 0.2
//max robot gerak = 0.58 m/s atau 2 km/jam

#define hold 169

const int angleM1 = 45;
const int angleM2 = 135;
const int angleM3 = 225;
const int angleM4 = 315;

float kRoda = M_PI*Droda;

volatile float prevErr_x,prevErr_y,prevErr_t;
volatile long prevTime;

float lx,ly,lt,deltaTime,
      passX,passY,errT,
      errX,errY,v1,v2,
      v3,v4,x,y,t;
volatile float errprevX,
      errprevY, errprevT;
int passT, in[50],sens,i;
string str;


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
    int receive_some,sonic,meg;
    std::string data;
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
            meg = atof(terima);
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

float toRad(float degree) {
  return degree * M_PI / 180;
}

Device atas("192.168.0.70",5555);
Device roda1("192.168.0.67",5555);
Device roda2("192.168.0.66",5555);
Device roda3("192.168.0.69",5555);
Device roda4("192.168.0.68",5555);

double calculateDeltaTime() {
    static auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> delta = end - start;
    return delta.count(); // Mengembalikan delta time dalam detik
}



struct Joystick
{
    bool connected;
    char buttonCount;
    float* bStat;
    char axisCount;
    float* axisStat;
    char name[128];
    int file;
};

Joystick openJoystick(const char* fileName)
{
    Joystick j = {0};
    int file = open(fileName, O_RDONLY | O_NONBLOCK);
    if (file != -1)
    {
        ioctl(file, JSIOCGBUTTONS, &j.buttonCount);
        j.bStat = (float*)calloc(j.buttonCount, sizeof(float));
        ioctl(file, JSIOCGAXES, &j.axisCount);
        j.axisStat = (float*)calloc(j.axisCount, sizeof(float));
        ioctl(file, JSIOCGNAME(sizeof(j.name)), j.name);
        j.file = file;
        j.connected = true;
    }
    return j;
}

void readJoystickInput(Joystick* joystick)
{
    while(1){
        js_event event;
        int bytesRead = read(joystick->file, &event, sizeof(event));
        if (bytesRead == 0 || bytesRead == -1) return;

        if (event.type == JS_EVENT_BUTTON && event.number < joystick->buttonCount) {
            joystick->bStat[event.number] = event.value ? 1:0;
        }
        if (event.type == JS_EVENT_AXIS && event.number < joystick->axisCount) {
            joystick->axisStat[event.number] = event.value;
        }
    }
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

void parsing(){
    atas.terimaData(sockfd);
    str = atas.data;
    string s;
    stringstream ss(str);
    vector<string> v;
    while (getline(ss, s, ',')) {
        v.push_back(s);
    }
    for (int i = 0; i < v.size(); i++) {
        in[i] = atoi(v[i].c_str());
    }
    sens = in[1];
}


void mkIkat(){
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