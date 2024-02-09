#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <termios.h>
#include <unistd.h>

// #include "icecream.hpp"

#include <ros/ros.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
using namespace std;

const int Left =37;
const int Up =38;
const int Right =39;
const int Down =40;
const int start =108;
const int stop =81;

std_msgs::String keys,y,x,out;
int8_t prevKeys,prevX,prevY,prevStart,prevStop,prevOut;

char getKeyPress() {
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

    return key;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "key");
    ros::NodeHandle nh;
    ros::Publisher keySend = nh.advertise<std_msgs::String>("sign",1000);
    ros::Rate loop_rate(100);
    ROS_INFO("start remote");
    while(ros::ok()){
        // ROS_INFO("%d",ros::Time::now);
        // cout<<ros::Time::now<<endl;
        char c = getKeyPress();
            if(c == 'A'){
                keys.data = "maju";
            }else if (c == 'B'){
                keys.data = "mundur";
            }else if(c == 'C'){
                keys.data = "kanan";
            }else if(c == 'D'){
                keys.data = "kiri";
            }else if(c =='S' || c =='s'){
                keys.data = "start";
            }else if(c == 'P' || c =='p'){
                keys.data = "pause";
            }else if(c == 'q' || c =='Q'){
                keys.data = "mati";
                exit(0);
            }else if( c == 'o'){
                keys.data = "putar kanan";
            }else if(c == 'b' || c == 'B'){
                keys.data = "Balik";
            }else if(c == 'q'){
                keys.data = "putar kiri";
            }
        keySend.publish(keys);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}