#ifndef CONTROL_H
#define CONTROL_H
#include "motion.h"
#include "std_msgs/Float32MultiArray.h"

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <linux/input.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <set>

using namespace std;

#define LeftHatX 0
#define LeftHatY 1
#define RightHatX 3
#define RightHatY 4
const int SILANG = 0;
const int CIRCLE = 1;
const int CIRCLE1 = -1;
const int SQUARE1 = -3;
const int TRIANGLE =2;
const int SQUARE =3;
const int L1 =4;
const int R1 =5;
const int L2 =2;
const int R2 =5;
const int ax_y =7; //axis < 0
// const int down 7 //axis >0
const int ax_x  =6; //axis >0
// const int right 6 //axis <0
const int triangle =2;//button
const int circle =1;//button
const int square =3;//button
// const int ex 0 //button
const int ps =10; //button
const int option =9;//button
const int share =8; //button


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


int roll,pitch,yaw,yawPure;
void callBackIMU(const std_msgs::Int32MultiArray::ConstPtr& pesanImu){
    roll = static_cast<float>(pesanImu -> data[0]);
    pitch = static_cast<float>(pesanImu -> data[1]);
    yaw = static_cast<float>(pesanImu -> data[2]);
    yawPure = static_cast<float>(pesanImu -> data[3]);
}

float enc1,enc2,enc3,enc4;
void callbackArrayMotor(const std_msgs::Int32MultiArray::ConstPtr& arrayMotor){
    enc1 = static_cast<float>(arrayMotor -> data[0]);
    enc2 = static_cast<float>(arrayMotor -> data[1]);
    enc3 = static_cast<float>(arrayMotor -> data[2]);
    enc4 = static_cast<float>(arrayMotor -> data[3]);
}

static int prevTanjakkan = 0;
static int tanjakkan = 0;
static int afterTanjakkan = 0;
const char* fileName = "/dev/input/js0";
Joystick joy = openJoystick(fileName);
static int speed;
int prevcase,caseBot;
void joyControl(){
    Kinematic motion;
    Point2D outF;
    motion.forward(enc1, enc2, enc3, enc4,yaw);
    readJoystickInput(&joy);
    //kecepatan
    static int axisCont = 0;
    static int prevAxisValue = 0;
    static int send_gripper_ball = 0;
    if (joy.axisStates[ax_y] > prevAxisValue && !axisCont) {
        speed--;
        axisCont =1;
    }
    if (joy.axisStates[ax_y] < prevAxisValue && !axisCont) {
        speed++;
        axisCont = 1;
    }else if(joy.axisStates[ax_y] == 0){
        axisCont = 0;
    }else if(speed >= 4){
        speed = 4;
    }else if(speed <= -1){
        speed = 0;
    }
    prevAxisValue = joy.axisStates[ax_y];
    static int silangPressed = 0;
    static int trianglePressed = 0;
    if (joy.buttonStates[SILANG] && !silangPressed) {
        if(prevcase < 8) {
            caseBot = prevcase + 1;
            prevcase = caseBot;
        }
        silangPressed = 1;
    }
    
    if (!joy.buttonStates[SILANG]) {
        silangPressed = 0;
    }
    if(joy.buttonStates[TRIANGLE] && !trianglePressed){
        if(prevcase > 1){
            caseBot = prevcase - 1;
            prevcase = caseBot;
        }
        trianglePressed =1;
    }
    if(!joy.buttonStates[TRIANGLE]){
        trianglePressed =0;
    }
    if (prevcase > 7) {
        caseBot = 7;
        prevcase = caseBot;
    }
    if(joy.buttonStates[ps]){
        caseBot = 0;
    }if(joy.buttonStates[option]){
        prevcase = 0;
    }
    ROS_INFO("%d,%d,%d\n,",speed,caseBot,prevcase);
}


void addData(int kasus) {
    Kinematic motion;
    motion.forward(enc1, enc2, enc3, enc4,yaw);
    ofstream file;
    ifstream readFile;
    stringstream filename;
    set<string> existingData;
    string line;

    if (kasus > 0 && kasus <= 12 && tanjakkan == 0) {
        if (kasus == 1) {
            filename << "data_cel/ambil1.txt";
        } else if (kasus == 2) {
            filename << "data_cel/ambil2.txt";
        } else if (kasus == 5) {
            filename << "data_cel/ambil3.txt";
        } else if (kasus == 6) {
            filename << "data_cel/ambil4.txt";
        } else if (kasus == 9) {
            filename << "data_cel/ambil5.txt";
        } else if (kasus == 10) {
            filename << "data_cel/ambil6.txt";
        } else if (kasus == 3) {
            filename << "data_cel/tanam1.txt";
        } else if (kasus == 4) {
            filename << "data_cel/tanam2.txt";
        } else if (kasus == 7) {
            filename << "data_cel/tanam3.txt";
        } else if (kasus == 8) {
            filename << "data_cel/tanam4.txt";
        } else if (kasus == 11) {
            filename << "data_cel/tanam5.txt";
        } else if (kasus == 12) {
            filename << "data_cel/tanam6.txt";
        }

        // Open the file for reading
        readFile.open(filename.str());
        if (!readFile.is_open()) {
            cerr << "Unable to open file for reading: " << filename.str() << endl;
            return;
        }
        
        // Read existing data from file
        while (getline(readFile, line)) {
            existingData.insert(line);
        }
        readFile.close();

        // Prepare the new data string
        string newData = to_string(motion.poseX) + "," + to_string(motion.poseY) + "," + to_string(motion.poseH);

        // Check if the new data is already in the set
        if (existingData.find(newData) == existingData.end()) {
            // Open the file for appending
            file.open(filename.str(), ios::app);
            if (!file.is_open()) {
                cerr << "Unable to open file for writing: " << filename.str() << endl;
                return;
            }
            file << newData << endl;
            file.close();
        }
    }
}

// Function to process each line and accumulate the sums for each column
void processLine(const string& line, float& sumX, float& sumY, int& sumT, int& count) {
    stringstream lineStream(line);
    string cell;
    vector<float> values;

    // Split the line by commas and convert to float
    while (getline(lineStream, cell, ',')) {
        values.push_back(stof(cell));
    }

    // Accumulate the sums for each column
    if (values.size() >= 3) {
        sumX += values[0];
        sumY += values[1];
        sumT += static_cast<int>(values[2]);
        count++;
    }
}


// Function to compute and return the averages based on type
vector<double> computeAverages(const string& filename) {
    ifstream inputFile(filename);
    vector<double> averages(3, -1.0);  // Initialize with -1.0 to indicate error

    if (!inputFile) {
        cerr << "Error opening file: " << filename << endl;
        return averages;
    }

    float sumX = 0.0;
    float sumY = 0.0;
    int sumT = 0;
    int count = 0;
    string line;

    while (getline(inputFile, line)) {
        processLine(line, sumX, sumY, sumT, count);
    }

    inputFile.close();

    if (count > 0) {
        averages[0] = sumX / count;
        averages[1] = sumY / count;
        averages[2] = static_cast<float>(sumT) / count;
    } else {
        cerr << "No data to process in file: " << filename << endl;
    }

    return averages;
}

#endif  