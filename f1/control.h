#ifndef CONTROL_H
#define CONTROL_H
#include "fethernet.h"
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <termios.h>
#include <unistd.h>
#include <string>

#define LeftHatX 0
#define LeftHatY 1
#define RightHatX 3
#define RightHatY 4
#define SILANG 0
#define CIRCLE 1
#define TRIANGLE 2
#define SQUARE 3
#define L1 4
#define R1 5
#define L2 2
#define R2 5

const int CIRCLE1 = -1;
const int SQUARE1 = -3;

#define MINSPEED 0.8
#define MAXSPEED 2
#define MAXSPEEDT 0.4
#define hold 169
#define startGripper 90

#define ax_y 7 //axis < 0
// #define down 7 //axis >0
#define ax_x  6 //axis >0
// #define right 6 //axis <0
#define triangle 2//button
#define circle 1//button
#define square 3//button
// #define ex 0 //button
#define ps 10 //button
#define option 9//button
#define share 8 //button

int caseBot,manual;
int prevcase;
int moveImu;
static int speed;
float speedChange[4] = {1.3,2,3,4};
float speedChangeT[4] = {1.5,2,3,4};

string getKeyPress() {
    char key;
    struct termios oldt, newt;

    // Save current terminal settings
    tcgetattr(STDIN_FILENO, &oldt);

    // Set terminal to non-canonical mode (no buffering)
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // Read a single character
    read(STDIN_FILENO, &key, 1);

    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return string(1, key);
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
// string key = getKeyPress();
// void keyboardControl(){
//     static int s_pressed = 0;
//     static int x_pressed = 0;
//     if(key == "s" && !s_pressed){
//         caseBot += 1;
//         s_pressed = 1;
//     }if(key != "s"){
//         s_pressed = 0;
//     }
//     if(key == "x" && !x_pressed){
//         caseBot += 1;
//         x_pressed = 1;
//     }if(key != "x"){
//         x_pressed = 0;
//     }
//     if(key == "A"){
//         //maju
//     }if(key == "B"){
//         //mundur
//     }if(key == "C"){
//         //kanan
//     }if(key == "D"){
//         //kiri
//     }
// }


static int prevTanjakkan = 0;
static int tanjakkan = 0;
static int afterTanjakkan = 0;
const char* fileName = "/dev/input/js0";
Joystick joy = openJoystick(fileName);
void inCaseAuto(){
    forKinematic();
    readJoystickInput(&joy);

    //kecepatan
    static int axisCont = 0;
    static int prevAxisValue = 0;
    static int send_gripper_ball = 0;
    if(pitch <= -5 && afterTanjakkan == 0){
        tanjakkan = prevTanjakkan + 1;
        prevTanjakkan = tanjakkan;

        afterTanjakkan = 1;
    }else if(pitch >= -1 && afterTanjakkan == 1){
        tanjakkan = prevTanjakkan + 1;
        prevTanjakkan = tanjakkan;
        afterTanjakkan = 2;
    }
    if(tanjakkan >= 1 && send_gripper_ball == 0){
        for(int i = 0; i <3000; i++){
            atas.kirimData(sockfd, std::to_string(39));
        }
        send_gripper_ball = 1;
    }
    // cout<<tanjakkan<<endl;
    if (joy.axisStat[ax_y] > prevAxisValue && !axisCont) {
        speed--;
        axisCont =1;
    }
    if (joy.axisStat[ax_y] < prevAxisValue && !axisCont) {
        speed++;
        axisCont = 1;
    }else if(joy.axisStat[ax_y] == 0){
        axisCont = 0;
    }else if(speed >= 4){
        speed = 3;

    }else if(speed <= -1){
        speed = 0;
    }
    prevAxisValue = joy.axisStat[ax_y];
    // Posisi
    static int silangPressed = 0;
    static int trianglePressed = 0;
    // Posisi
    if (joy.bStat[SILANG] && !silangPressed) {
        if(prevcase < 13) {
            caseBot = prevcase + 1;
            prevcase = caseBot;
        }
        silangPressed = 1;
    }
    // Reset button flags if buttons are released
    if (!joy.bStat[SILANG]) {
        silangPressed = 0;
    }
    if(joy.bStat[TRIANGLE] && !trianglePressed){
        if(prevcase > 1){
            caseBot = prevcase - 1;
            prevcase = caseBot;
        }
        trianglePressed =1;
    }
    if(!joy.bStat[TRIANGLE]){
        trianglePressed =0;
    }
    if (prevcase == 13) {
        caseBot = 12;
        prevcase = caseBot;
    }
    if(joy.bStat[ps]){
        manual = 0;
        caseBot = 0;
    }
    if(joy.bStat[option]){
        prevcase = 0;
        afterTanjakkan = 0;
        prevTanjakkan = 0;
        tanjakkan = 0;
        send_gripper_ball = 0;
        atas.kirimData(sockfd, std::to_string(999));
    }
    int lamp[4] = {101,102,103,104};
    switch (speed)
    {
    case 0:
        atas.kirimData(sockfd, std::to_string(lamp[0]));
        break;
    case 1:
        atas.kirimData(sockfd, std::to_string(lamp[1]));
        break;
    case 2:
        atas.kirimData(sockfd, std::to_string(lamp[2]));
        break;
    case 3:
        atas.kirimData(sockfd, std::to_string(lamp[3]));
        break;
    }
    // printf("caseBot = %d\nprevCase =%d\n",caseBot,prevcase);
}
// void tanjakkan(){
//     if(tanjakkan ==1){
//         adjust(hx, hy, 45, passT);
//     }else {
//         inKinematic(hx,hy,ht, passT); 
//     }
// }
#endif
