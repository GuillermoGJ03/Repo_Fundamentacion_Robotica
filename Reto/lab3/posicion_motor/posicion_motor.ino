#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

int sentido1 = 8;
int sentido2 = 9;
int encoderA = 2; //Amarillo
int encoderB = 3; // Verde

int enable = 5;
const uint8_t potenciometro = A0;
uint16_t val = 0;
long int pulsos = 0; 
float pos = 0.0;

long int tiempo_anterior = 0;
long int tiempo_actual;
long int tiempo = 2500;
float posicion_anterior = 0;
float posicion_actual;
float rpm = 0.0;

uint8_t sentido;

bool encoderA_state, encoderB_state;

ros::NodeHandle nh;

void pwmCallback(std_msgs::Int16 &msg){
  int16_t pwm = msg.data;
  if(pwm < 0){
    digitalWrite(sentido1, LOW);
    digitalWrite(sentido2, HIGH);
  } else if(pwm > 0){
    digitalWrite(sentido1, HIGH);
    digitalWrite(sentido2, LOW);    
  } else{
    digitalWrite(sentido1, LOW);
    digitalWrite(sentido2, LOW);
    analogWrite(enable, 0);    
  }
  analogWrite(enable, abs(pwm));
}

std_msgs::Float32 flPOS_msg;
std_msgs::Float32 flRPM_msg;

ros::Publisher POS("POS", &flPOS_msg);
ros::Publisher RPM("RPM", &flRPM_msg);

ros::Subscriber<std_msgs::Int16> pwm("pwm", &pwmCallback);

void setup() {
  nh.initNode();
  nh.advertise(POS);
  nh.advertise(RPM);
  nh.subscribe(pwm);

  pinMode(sentido1, OUTPUT);
  pinMode(sentido2, OUTPUT);
  pinMode(enable, OUTPUT);
  pinMode(potenciometro, INPUT);

  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA), estado_encoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderB), comprobacion_encoderB, RISING);
}

void loop() {
  
  //val = analogRead(potenciometro);
  pos = (float)pulsos/495.0;

  flPOS_msg.data = pos;
  flRPM_msg.data = rpm;
  
  POS.publish(&flPOS_msg);
  RPM.publish(&flRPM_msg);

  nh.spinOnce();
  delay(500);
}

void estado_encoderA(){
  encoderA_state = digitalRead(encoderA);
  if(encoderA_state == 1) {
    tiempo_actual = micros();
    tiempo = tiempo_actual-tiempo_anterior;
    tiempo_anterior = tiempo_actual;
    if(tiempo >= 2400){
      rpm = 0.0;
      tiempo = 2500;
    } else{
      rpm =  (60000000.0)/(float(tiempo)*495);
      tiempo = 2500;
    } 
  }
}

void comprobacion_encoderB(){
    encoderB_state = digitalRead(encoderB);
    if(encoderA_state == encoderB_state){
      sentido = 1;
      pulsos++;

    }else if(encoderA_state != encoderB_state){
      sentido = -1;
      pulsos--;
    }
}