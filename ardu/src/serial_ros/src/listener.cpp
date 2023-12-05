#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"

void gyro_ambil(const geometry_msgs::Quaternion::ConstPtr &msg){
    float ix = msg->x;
    float iy = msg->y;
    float iz = msg->z;
    ROS_INFO("%f,%f,%f", ix,iy,iz);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("chit_chat", 1000, gyro_ambil);
  // ros::Publisher p = n.advertise<geometry_msgs::Float64>("gc", 1000);
  /*subscribe ke chatter topic dengan masternya. ros akan memanggil chatterCallback()
  fungsi kapan pun pesan baru sampai. dalam kasus ini jika antrian sampai 1000 pesan
  kita akan mulai membuang pesan lama*/
  
  ros::spin();
  // while(ros::ok())
  // {
  //   // float gyro = datagyro;
  //   // gyro *= sin(45);
  //   // std_msgs::Float32 gyro_msg;
  //   // gyro_msg.data = gyro;
  //   // p.publish(gyro_msg);
  //   ros::spinOnce();
  // }
  /*memasuki loop dan akan memangil panggilan kembali secepat mungkin. ros::spin()
  akan berhenti jika ros::ok() kembali salah.*/
  return 0;
}