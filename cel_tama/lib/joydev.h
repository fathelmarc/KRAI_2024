#ifndef JOYDEV_H
#define JOYDEV_H
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <linux/input.h>


#define LeftHatX 0
#define LeftHatY 1
#define RightHatX 3
#define RightHatY 4
#define silang 0
#define circle 1
#define triangle 2
#define square 3
#define l1 4
#define r1 5
#define l2 2
#define r2 5

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

struct Joystick
{
    bool connected;
    char buttonCount;
    float* buttonStates;
    char axisCount;
    float* axisStates;
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
		j.buttonStates = (float*)calloc(j.buttonCount, sizeof(float));
		ioctl(file, JSIOCGAXES, &j.axisCount);
		j.axisStates = (float*)calloc(j.axisCount, sizeof(float));
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
            joystick->buttonStates[event.number] = event.value ? 1:0;
        }
        if (event.type == JS_EVENT_AXIS && event.number < joystick->axisCount) {
            joystick->axisStates[event.number] = event.value;
        }
    }
}
// int main()
// {
// 	const unsigned int maxJoysticks = 32;
// 	Joystick joysticks[maxJoysticks] = {0};

// 	char fileName[32];
// 	for (unsigned int i=0; i<maxJoysticks; ++i)
// 	{
// 		sprintf(fileName, "/dev/input/js%d", i);
// 		joysticks[i] = openJoystick(fileName);
// 	}

// 	while (1)
// 	{
// 		for (unsigned int i=0; i<maxJoysticks; ++i)
// 		{
// 			if (joysticks[i].connected)
// 			{
// 				readJoystickInput(&joysticks[i]);

// 				printf("%s - Axes: ", joysticks[i].name);
// 				for (char axisIndex=0; axisIndex<joysticks[i].axisCount; ++axisIndex) {
// 					printf("%d:% 6d ", axisIndex, joysticks[i].axisStates[axisIndex]);
// 				}
// 				printf("Buttons: ");
// 				for (char buttonIndex=0; buttonIndex<joysticks[i].buttonCount; ++buttonIndex) {
// 					if (joysticks[i].buttonStates[buttonIndex]) printf("%d ", buttonIndex);
// 				}
// 				printf("\n");
// 			}
// 		}
// 		fflush(stdout);
// 		usleep(16000);
// 	}
// }
#endif