#include <ros.h>
#include <std_msgs/UInt16.h>

const int potenciometro = A0;

ros::NodeHandle nh;

std_msgs::UInt16 uint16_msg;
ros::Publisher pot("pot", &uin168_msg);

uint16_t val = 0;

void setup() {
  nh.initNode();
  nh.advertise(pot);
}

void loop() {
  val = analogRead(potenciometro);
  uint8_msg.data = val;
  pot.publish(&uint16_msg);
  nh.spinOnce();
  delay(1);  
}
