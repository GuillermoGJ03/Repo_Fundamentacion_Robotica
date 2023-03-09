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
int encoderA_sate, encoderB_state;

// Respuestas del motor
int sentido = 0;
long int pulsos = 0;
float posicion = 0.0;
float rpm = 0.0;

// Tiempos entre pulsos
unsigned long tiempo_anterior = 0;
unsigned long tiempo_actual = 0;
unsigned long tiempo;

// Callback del subscriber
void pwmCallback(const std_msgs::Int16 & msg){
  int16_t pwm_signal = abs(msg.data);
  if(pwm > 0){
    digitalWrite(SENTIDO_1, LOW);
    digitalWrite(SENTIDO_2, HIGH);
  } else if(pwm < 0){
    digitalWrite(SENTIDO_1, HIGH);
    digitalWrite(SENTIDO_2, LOW);
  } else{
    digitalWrite(SENTIDO_1, LOW);
    digitalWrite(SENTIDO_2, LOW);
    pwm_signal = 0;
    rpm = 0;
  }
  analogWrite(ENABLE, pwm_signal);
}

// Variables de ros
ros::NodeHandle nh;

std_msgs::Float32 flPOS_msg;
std_msgs::Float32 flRPM_msg;

ros::Publisher POS("POS", &flPOS_msg);
ros::Publisher RPM("RPM", &flRPM_msg);

ros::Subscriber<std_msgs::Int16> PWM("PWM", &pwmCallback);

ros::Publisher POS("POS", &flPOS_msg);
ros::Publisher RPM("RPM", &flRPM_msg);

void setup(){
  // Señales para el motor
  pinMode(ENABLE, OUTPUT);
  pinMode(SENTIDO_1, OUTPUT);
  pinMode(SENTIDO_2, OUTPUT);

  // Señales e interrupciones del encoder
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), estado_encoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), comprobacion_encoderB, RISING);

  // Inicializacion de ROS
  nh.initNode();
  nh.advertise(POS);
  nh.advertise(RPM);
  nh.subscribe(PWM);
}

void loop(){
  flPOS_msg.data = posicion;
  flRPM_msg.data = rpm * sentido;
  
  POS.publish(&flPOS_msg);
  RPM.publish(&flRPM_msg);

  nh.spinOnce();
  delay(1);
}

void estado_encoderA(){
  encoderA_state = digitalRead(ENCODER_A);
  if(encoderA_state == 1){
    tiempo_actual = micros();
    tiempo = tiempo_actual-tiempo anterior;
    tiempo_anterior = tiempo_actual;
    rpm = 60000000.0/((float)tiempo*495.0)
  }

}

void comprobacion_encoderB(){
  encoderB_state = digitalREad(ENCODER_B)
  if(encoderA_state == encoderB_state){
    sentido = 1; // Clockwise
    if(encoderB_sate == 1){
      pulsos++;
    }
  }else if(encoderA_state != encoderB_state){
    sentido = -1; // Counter-Clockwise
    if(encoderB_state == 1){
      pulsos--;
    }
  }
  posicion = (float)pulsos/495.0;
}