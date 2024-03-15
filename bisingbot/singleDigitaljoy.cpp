#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <linux/input.h>

int caseBot;

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

const char* fileName = "/dev/input/js0"; // Anggap kita hanya memiliki satu joystick dan itu adalah js0

Joystick joy = openJoystick(fileName);
void inCaseAuto(){
    readJoystickInput(&joy);
    if(joy.bStat[0]){
        caseBot = 2;
    }else if(joy.bStat[2]){
        caseBot = 3;
    }
}
int main()
{
    if (!joy.connected) {
        printf("joy tidak terhubung\n");
        return 1;
    }
    while (1){   
        inCaseAuto();
        printf("%d\n",caseBot);
		fflush(stdout);
		usleep(16000);
    }

    return 0;
}

