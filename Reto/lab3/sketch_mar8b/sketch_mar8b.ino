#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

#define SENTIDO_1 8
#define SENTIDO_2 9
#define ENCODER_A 2 //Amarillo
#define ENCODER_B 3 //Verde
#define ENABLE    5

bool encoderA_state, encoderB_state;

int sentido = 0;
long int pulsos = 0;
float posicion = 0.0;
float rpm = 0.0;

unsigned long tiempo_anterior = 0;
unsigned long tiempo_actual = 0;
unsigned long tiempo = 0;

void pwmCallback(const std_msgs::Int16 & msg){
  int16_t pwm = msg.data;
  if(pwm < 0){
    analogWrite(ENABLE, abs(pwm));
    digitalWrite(SENTIDO_1, LOW);
    digitalWrite(SENTIDO_2, HIGH);
  }else if(pwm > 0){
    analogWrite(ENABLE, abs(pwm));
    digitalWrite(SENTIDO_1, HIGH);
    digitalWrite(SENTIDO_2, LOW);
  }else{
    analogWrite(ENABLE, 0);
    digitalWrite(SENTIDO_1, LOW);
    digitalWrite(SENTIDO_2, LOW);    
    rpm = 0.0;
    sentido = 0;  
  }
  
}

ros::NodeHandle nh;

std_msgs::Float32 flPOS_msg;
std_msgs::Float32 flRPM_msg;

ros::Publisher POS("POS", &flPOS_msg);
ros::Publisher RPM("RPM", &flRPM_msg);

ros::Subscriber<std_msgs::Int16> PWM("PWM", &pwmCallback);

void setup() {
  nh.initNode();
  nh.advertise(POS);
  nh.advertise(RPM);
  nh.subscribe(PWM);

  pinMode(SENTIDO_1, OUTPUT);
  pinMode(SENTIDO_2, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  //pinMode(potenciometro, INPUT);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), estado_encoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), comprobacion_encoderB, CHANGE);
}

void loop() {
  posicion = (float)pulsos/495.0;

  flPOS_msg.data = posicion;
  flRPM_msg.data = rpm;
  
  POS.publish(&flPOS_msg);
  RPM.publish(&flRPM_msg);

  nh.spinOnce();
  delay(1);
  
  /*if(sentido == 1){
    Serial.println("Clockwise");
  }else if(sentido == -1){
    Serial.println("Counter-Clockwise");
  }
  Serial.println(posicion);*/

  
}

void estado_encoderA(){
  encoderA_state = digitalRead(ENCODER_A);
  if(encoderA_state == 1){
    tiempo_actual = micros();
    tiempo = tiempo_actual - tiempo_anterior;
    tiempo_anterior = tiempo_actual;
    rpm = (60000000.0)/((float)tiempo*495.0)*(sentido);
  }
}

void comprobacion_encoderB(){
    encoderB_state = digitalRead(ENCODER_B);
    if(encoderA_state == encoderB_state){
      sentido = 1;
      pulsos++;
    }else if(encoderA_state != encoderB_state){
      sentido = -1;
      pulsos--;
    }
}