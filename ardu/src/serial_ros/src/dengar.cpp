#include "ros/ros.h"
#include "std_msgs/Float32.h"

float gyro = 0;

void panggil(const std_msgs::Float32& msg){
    gyro = msg.data;
    ROS_INFO("%f", gyro);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "dengar");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("chatter",1000,panggil);

    ros::spin();
    return 0;
}