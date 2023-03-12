#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

// Pines de enconder y motor
#define ENCODER_A 2 //Amarillo
#define ENCODER_B 3 //Verde
#define ENABLE    5
#define SENTIDO_1 8
#define SENTIDO_2 9

// Estado de los encoders
int encoderA_state, encoderB_state, prev_state = 0;

// Respuestas del motor
int sentido = 0;
long int pulsos = 0;
float posicion = 0.0;
float rpm = 0.0;

// Tiempos entre pulsos
unsigned long tiempo_anterior = 0;
unsigned long tiempo_actual = 0;
unsigned long tiempo;

// Variables de ros
ros::NodeHandle nh;

std_msgs::Float32 flPOS_msg;
std_msgs::Float32 flRPM_msg;

// Callback del subscriber
void pwmCallback(const std_msgs::Int16 & msg){
  int16_t pwm_signal;
  if(msg.data > 0){
    digitalWrite(SENTIDO_1, LOW);
    digitalWrite(SENTIDO_2, HIGH);
    pwm_signal = msg.data;
  } else if(msg.data < 0){
    digitalWrite(SENTIDO_1, HIGH);
    digitalWrite(SENTIDO_2, LOW);
    pwm_signal = abs(msg.data);
  } else{
    digitalWrite(SENTIDO_1, LOW);
    digitalWrite(SENTIDO_2, LOW);
    pwm_signal = 0;
    rpm = 0;
  }
  analogWrite(ENABLE, pwm_signal);
}

ros::Publisher spin_pub("spin_pub", &flPOS_msg);
ros::Publisher rpm_pub("spm_pub", &flRPM_msg);

ros::Subscriber<std_msgs::Int16> pwm_listener("pwm_listener", &pwmCallback);

void setup(){
  // Señales para el motor
  pinMode(ENABLE, OUTPUT);
  pinMode(SENTIDO_1, OUTPUT);
  pinMode(SENTIDO_2, OUTPUT);

  // Señales e interrupciones del encoder
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), estado_encoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), comprobacion_encoderB, CHANGE);

  // Inicializacion de ROS
  nh.initNode();
  nh.advertise(spin_pub);
  nh.advertise(rpm_pub);
  nh.subscribe(pwm_listener);
}

void loop(){
  flPOS_msg.data = posicion;
  flRPM_msg.data = rpm * sentido;
  
  spin_pub.publish(&flPOS_msg);
  rpm_pub.publish(&flRPM_msg);
  
  nh.spinOnce();
  delay(10);
}

void estado_encoderA(){
  encoderA_state = digitalRead(ENCODER_A);
  if(encoderA_state == 1){
    tiempo_actual = micros();
    tiempo = tiempo_actual-tiempo_anterior;
    tiempo_anterior = tiempo_actual;
    rpm = 60000000.0/((float)tiempo*495.0);
  }  
}

void comprobacion_encoderB(){
  encoderB_state = digitalRead(ENCODER_B);
  if(encoderB_state != prev_state){
    if(encoderA_state == encoderB_state){
      sentido = -1; // Clockwise
      if(encoderB_state == 1){
        pulsos++;
      }
    }else if(encoderA_state != encoderB_state){
      sentido = 1; // Counter-Clockwise
      if(encoderB_state == 1){
        pulsos--;
      }
    }
    posicion = (float)pulsos/495.0;
  }
  prev_state = encoderB_state;
}