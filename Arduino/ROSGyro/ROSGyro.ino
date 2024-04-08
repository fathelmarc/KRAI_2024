#include <ros.h>
#include <geometry_msgs/Quaternion.h>

ros::NodeHandle nh;
geometry_msgs::Quaternion pImu;

ros::Publisher pub_imu("imu", &pImu);

float imu;
struct gy25 {
  char buffer[50];
  int counter;
  float heading;
  float roll;
  float pitch;
} cmps;

void setup() {
  nh.initNode();
  Serial.begin(57600);
  nh.advertise(pub_imu);
  calibration();
}

void loop() {
  updateCMPS();
  imu = cmps.heading;
  if (imu > 0){
    float selisih = 180 -imu;
    imu = 180 + selisih;
    if (imu > 360){
      imu = 0;
    }
  }
  if (imu <0){
    imu = imu * -1;
  }
  Serial.println(imu);
  pImu.x = imu;
  pub_imu.publish(&pImu);
  nh.spinOnce();
  delay(1);
}

void updateCMPS() {
  char tmp; // Variabel temporary
  while (Serial3.available()) {
    tmp = Serial3.read();
    cmps.buffer[cmps.counter++] = tmp;
    if (tmp == '\n') { // Langkah 1
      cmps.buffer[cmps.counter] = 0; // Karakter terminator
      cmps.heading = atof(strtok(cmps.buffer + 5, ","));
      cmps.counter = 0;
    }
  }
}

void calibration() {
  Serial3.begin(115200); // Serial GY25
  delay(3000); // Jeda 3 detik

  // Kalibrasi Tilt
  Serial3.write(0xA5);
  Serial3.write(0x54);

  delay(1000); // Jeda sebelum kalibrasi heading

  // Kalibrasi Heading
  Serial3.write(0xA5);
  Serial3.write(0x55);

  delay(100); // Jeda sebelum konfigurasi output

  // Output ASCII
  Serial3.write(0xA5);
  Serial3.write(0x53);

  delay(100); // Jeda sebentar
}
