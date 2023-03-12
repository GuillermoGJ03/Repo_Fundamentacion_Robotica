#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

//Lo que vas a mandar
std_msgs::String str_msg;
std_msgs::Int32 int32_msg;

void boolCallback(std_msgs::Bool &msg)
{
  digitalWrite(13,msg.data);
}

ros::Publisher str_chatter("str_chatter", &str_msg);
ros::Publisher num_chatter("num_chatter", &int32_msg);

//Subsciber que va a recibir mensajes de tipo bool
ros::Subscriber<std_msgs::Bool> bool_listener("bool_listener", &boolCallback);

int value = 0;
char str_1[10] = "un espacio";

void setup() {
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(str_chatter);
  nh.advertise(num_chatter);
  nh.subscribe(bool_listener);
}

void loop() {
  str_msg.data = str_1;
  str_chatter.publish(&str_msg);

  int32_msg.data = value;
  num_chatter.publish(&int32_msg);

  value++;
  nh.spinOnce();
  delay(500);
}
