#include <ros/ros.h>
#include <std_msgs/Empty.h>

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "heartbeat_node");
    ros::NodeHandle nh;

    // Publisher for the heartbeat topic
    ros::Publisher heartbeat_pub = nh.advertise<std_msgs::Empty>("heartbeat", 10);

    // Define the loop rate (e.g., 1 Hz)
    ros::Rate loop_rate(1);

    while (ros::ok()) {
        // Create an empty message
        std_msgs::Empty msg;

        // Publish the heartbeat message
        heartbeat_pub.publish(msg);

        // Spin once to handle callbacks
        ros::spinOnce();

        // Sleep for the remaining time to maintain the loop rate
        loop_rate.sleep();
    }

    return 0;
}
