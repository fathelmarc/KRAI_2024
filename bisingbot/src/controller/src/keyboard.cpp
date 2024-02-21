#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <termios.h>
#include <unistd.h>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
using namespace std;

std_msgs::Int32 ang;
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
    ros::Publisher angkaSend = nh.advertise<std_msgs::Int32>("angka",1000);
    ros::Rate loop_rate(2000);
    ROS_INFO("start remote");
    while(ros::ok()){
        // ROS_INFO("%d",ros::Time::now);
        // cout<<ros::Time::now<<endl;
        char c = getKeyPress();
            if(c == 'A'){
                keys.data = "maju";
                // ROS_INFO("maju");
            }else if (c == 'B'){
                keys.data = "mundur";
                // ROS_INFO("mundur");
            }else if(c == 'C'){
                keys.data = "kanan";
                // ROS_INFO("kanan");
            }else if(c == 'D'){
                keys.data = "kiri";
                // ROS_INFO("kiri");
            }else if(c =='S' || c =='s'){
                keys.data = "start";
                // ROS_INFO("START");
            }else if(c == 'P' || c =='p'){
                keys.data = "pause";
                // ROS_INFO("pause");
            }else if(c == 'x' || c =='X'){
                keys.data = "mati";
                exit(0);
            }else if( c == 'o'){
                keys.data = "putar kanan";
            }else if(c == 'b' || c == 'B'){
                keys.data = "Balik";
            }else if(c == 'k'){
                keys.data = "putar kiri";
            }else if(c == 'q'){
                keys.data = "q";
                // ROS_INFO("q");
            }else if(c == 'w'){
                keys.data = "w";
                // ROS_INFO("w");
            }else if(c == 'e'){
                keys.data = "e";
                // ROS_INFO("e");
            }else if(c == 'r'){
                keys.data = "r";
                // ROS_INFO("r");
            }else if(c == 't'){
                keys.data = "t";
                // ROS_INFO("t");
            }else if(c == 'y'){
                keys.data = "y";
                // ROS_INFO("y");
            }
        keySend.publish(keys);
        angkaSend.publish(ang);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
