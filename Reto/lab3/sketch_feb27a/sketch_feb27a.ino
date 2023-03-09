/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

// Use the following line if you have a Leonardo or MKR1000
//#define USE_USBCON

// Importar librerias dependiendo del tipo de datos que vas a utilizar
#include <ros.h>
#include <std_msgs/String.h>

// Crear node handle (permite crear publishers y suscribers)
// Tambien se encarga del manejor de los puertos
ros::NodeHandle nh;

// Se crear el publisher que se estara usando-
// Este tien el topic de nombre "chatter"
// La segunda linea es un parametro para Publisher,  
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}