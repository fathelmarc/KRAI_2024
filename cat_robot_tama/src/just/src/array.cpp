#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
  ROS_INFO("Received array:");
  for (size_t i = 0; i < msg->data.size(); i++) {
    ROS_INFO("  Element %lu: %d", i, msg->data[i]);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "array_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("array_topic", 1000, arrayCallback);

  ros::spin();
  return 0;
}
