#define IN_1_PIN          8
#define IN_2_PIN          9
#define ENABLE_PIN        10

#define ENCODER_A_PIN     2
#define ENCODER_B_PIN     3

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

//uint16_t potentiometer_value = 0;
float pwm_signal = 0.0;

int direction = 1;
bool encoderA_state = 0;
unsigned long current_pulse_time = 0;
bool encoderB_state = 0;
unsigned long previous_pulse_time = 0;
long int encoder_pulses = 0;
float encoder_time = 0.0;
float encoder_rpm = 0.0;
float encoder_position = 0.0;

void pwm_Callback(const std_msgs::Int16 & msg)
{
  if (msg.data > 0)
  {
    pwm_signal = msg.data;
    digitalWrite(IN_1_PIN, HIGH);
    digitalWrite(IN_2_PIN, LOW);
  } else if (msg.data < 0)
  {
    pwm_signal = abs(msg.data);
    digitalWrite(IN_2_PIN, HIGH);
    digitalWrite(IN_1_PIN, LOW);
  } else 
  {
    pwm_signal = 0;
    encoder_rpm = 0;
    direction = 0;
  }
  analogWrite(ENABLE_PIN, pwm_signal);
}

std_msgs::Float32 position_msg;
ros::Publisher position_publisher("position_publisher", &position_msg);
std_msgs::Float32 rpm_msg;
ros::Publisher rpm_publisher("rpm_publisher", &rpm_msg);

ros::Subscriber<std_msgs::Int16> pwm_subscriber("pwm_subscriber", &pwm_Callback);


void setup() {
  pinMode(IN_1_PIN, OUTPUT);
  digitalWrite(IN_1_PIN, HIGH);
  pinMode(IN_2_PIN, OUTPUT);
  digitalWrite(IN_2_PIN, LOW);
  pinMode(ENABLE_PIN, OUTPUT);

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderA_Callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoderB_Callback, CHANGE);
  
  nh.initNode();
  nh.advertise(position_publisher);
  nh.advertise(rpm_publisher);
  nh.subscribe(pwm_subscriber);  
}

void loop()
{
  if (direction == 0)
  {
    encoder_rpm = encoder_rpm*(-1);
  }


  rpm_msg.data = encoder_rpm;
  rpm_publisher.publish( &rpm_msg );
  position_msg.data = encoder_position; 
  position_publisher.publish( &position_msg );

  nh.spinOnce();
  delay(10);
}

void encoderA_Callback()
{
  encoderA_state = digitalRead(ENCODER_A_PIN);
  current_pulse_time =  micros();

  encoder_time = (current_pulse_time - previous_pulse_time);

  previous_pulse_time = current_pulse_time;
  encoder_rpm = 60000000.0/(encoder_time*11.0*45.0) * direction;
}

void encoderB_Callback()
{
  encoderB_state = digitalRead(ENCODER_B_PIN);
  encoder_position = encoder_pulses/(11.0 * 45.0);

  if (encoderA_state == encoderB_state)
  {
    direction = 1;
    encoder_pulses++;
  } else if (encoderA_state != encoderB_state)
  {
    direction = -1;
    encoder_pulses--;
  }

}