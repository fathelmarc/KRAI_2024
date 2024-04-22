#ifndef CONTROL_H
#define CONTROL_H
#include "fethernet.h"
#include "motion.h"
#define LeftHatX 0
#define LeftHatY 1
#define RightHatX 3
#define RightHatY 4
#define SILANG 0
#define CIRCLE 1
const int CIRCLE1 = -1;
#define TRIANGLE 2
#define SQUARE 3
const int SQUARE1 = -3;
#define L1 4
#define R1 5
#define L2 2
#define R2 5
#define butY 7 
#define butX 6
#define UP 11
#define DOWN -11
#define MINSPEED 0.8
#define MAXSPEED 2
#define MAXSPEEDT 0.4
#define hold 169


int caseBot,manual;
int prevcase;
int moveImu;
static int speed;
float speedChange[4] = {70,100,150};
float speedChangeT[4] = {70,100,150};

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

const char* fileName = "/dev/input/js0";
Joystick joy = openJoystick(fileName);
void inCaseAuto(){
    forKinematic();
    readJoystickInput(&joy);

    //kecepatan
    static int axisCont = 0;
    static int prevAxisValue = 0;
    if (joy.axisStat[7] > prevAxisValue && !axisCont) {
        speed--;
        axisCont =1;
    }
    if (joy.axisStat[7] < prevAxisValue && !axisCont) {
        speed++;
        axisCont = 1;
    }else if(joy.axisStat[7] == 0){
        axisCont = 0;
    }else if(speed >= 4){
        speed = 3;

    }else if(speed <= -1){
        speed = 0;
    }
    prevAxisValue = joy.axisStat[7];
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
    if(joy.bStat[10]){
        manual = 0;
        caseBot = 0;
    }else if(joy.bStat[9] || joy.bStat[8]){
        prevcase = 0;
    }

    if(joy.bStat[7]){
        moveImu= 1;
    }else if(joy.bStat[6]){
        moveImu = 0;
    }
    switch (speed)
    {
    case 0:
        atas.kirimData(sockfd,std::to_string(101));
        break;
    case 1:
        atas.kirimData(sockfd,std::to_string(102));
        break;
    case 2:
        atas.kirimData(sockfd,std::to_string(103));
        break;
    case 3:
        atas.kirimData(sockfd,std::to_string(104));
        break;
    }
    // printf("caseBot = %d\nprevCase =%d\n",caseBot,prevcase);
}
#endif