// Variables de ROS
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

// Variables de arduino
int valor;
const int channelPinA = 2;
const int channelPinB = 3;
int valorAnterior = 0;
int valorActual;
int value_channelB;
int estado;
long int pulsos;
float spin;
float rpm;
unsigned long time_N = 0, timeV = 0, timeV_past = 0; 
int pwm_data;

ros::NodeHandle nh;

std_msgs::Float32 float32_msg;
std_msgs::Float32 float32_msg2;
std_msgs::Int16 int32_msg;

void pwm_listenerCallback(const std_msgs::Int16& msg)
{
  if (msg.data > 0){
  pwm_data = msg.data;
  digitalWrite(8,LOW);
  digitalWrite(9,HIGH);
  }else if (msg.data < 0){
  pwm_data = abs(msg.data);
  digitalWrite(8,HIGH);
  digitalWrite(9,LOW);
  }else{
    rpm =0;
    pwm_data = 0;
  }
  analogWrite(5,pwm_data);
}

ros::Publisher spin_pub("spin_pub", &float32_msg);
ros::Publisher rpm_pub("rpm_pub", &float32_msg2);

ros::Subscriber<std_msgs::Int16>pwm_listener("pwm_listener", &pwm_listenerCallback );


void setup() {
  // put your setup code here, to run once:
  pinMode(5, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  // encoder pines
  pinMode(channelPinA, INPUT_PULLUP);
  pinMode(channelPinB, INPUT_PULLUP);
  

  // Interrupt
  attachInterrupt(digitalPinToInterrupt(channelPinA), doEncodeA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channelPinB), doEncodeB, CHANGE);
  // ROS
  nh.initNode();
  nh.advertise(spin_pub);
  nh.advertise(rpm_pub);
  nh.subscribe(pwm_listener);
}

void loop() {
  float32_msg.data = spin;
  spin_pub.publish( &float32_msg );
  float32_msg2.data = estado * rpm;
  rpm_pub.publish( &float32_msg2 );
  nh.spinOnce();
  
  delay(10);  
}

void doEncodeA()
{
  valorActual = digitalRead(channelPinA); 
  // Antirebote para saber si este no se mantiene en 1 1 o 0 0, si no son iguales se hace el programa
  if (valorActual != valorAnterior) {
 
    // Serial.println("Son distintos");
    if (valorActual == value_channelB)
      {
        // Izquierda
        estado = -1;
        if(valorActual == 1){
          pulsos++;   

        } 

      }
    if (valorActual != value_channelB)
      {
        // Derecha
        estado = 1;
        if(valorActual == 1){
          pulsos--;
        }
      }
      spin = pulsos/11.0 * 45.0;
  }  
  else{
    // Serial.println("Son iguales");
  }
  valorAnterior = valorActual;
}

void doEncodeB()
{
  value_channelB = digitalRead(channelPinB); 
  // NOS ASEGURAMOS QUE SOLO MIDA UNA SUBIDA, ESTO DEBIDO A QUE CHANGE ENTRA DOBLE VES
  if(value_channelB == 1){
    timeV = micros();
    time_N = (timeV - timeV_past);
    timeV_past = timeV;
    if(time_N >= 4000){
      rpm = 0;
    }else{
      rpm = (60000000.0/((float)time_N * 11.0 * 45.0 ));
    }
  }
}


