#include <ros.h>
#include <std_msgs/Int32.h>
#include <PID_v1.h>
#include <math.h>
#include "Timer.h"
#include <IRremote.h>

#define IN1 8                 // control pin for left motor
#define IN2 9                 // control pin for left motor
#define IN3 10                // control pin for right motor
#define IN4 11                // control pin for right motor
#define ENA 5                 // analog pin for motor PWM
#define ENB 6                 // analog pin for motor PWM

#define touch_pin_L 12        // connected to mid pin of touch sensor
#define touch_pin_R 13        // connected to mid pin of touch sensor
#define touch_pin_M A1        // connected to mid pin of touch sensor

#define photo_sensor_pin A0

int val = 0;
const byte encoder0pinA = 2;  // A pin -> the interrupt pin 0 (pin 2)
const byte encoder0pinB = 7;  // B pin -> the digital pin 7
const byte encoder1pinA = 3;  // A pin -> the interrupt pin 1 (pin 3)
const byte encoder1pinB = 4;  // B pin -> the digital pin 4

int start = 0;

Timer t;
const int RECV_PIN = A2;
IRrecv irrecv(RECV_PIN);
int zero_counter = 0;
int all_counter = 0;

/* structure of encoder */
typedef struct
{
  int duration;
  byte encoderPinALast;
  boolean Direction;
}encoder_two_wheel;

/* declare variables for encoder */
encoder_two_wheel encoder_left;
encoder_two_wheel encoder_right;

/* structure of PID */
typedef struct
{
  double val_output;
  double setpoint;
  double Kp;
  double Ki;
  double Kd;
  double abs_duration;
  boolean result;
  boolean forward_backward;
}pid_two_wheel;

/* declare variables for PID control */
pid_two_wheel pid_left = {
  .val_output = 0.0,
  .setpoint = 0.0,
  .Kp = 0.6,
  .Ki = 5,
  .Kd = 0.0,
  .abs_duration = 0.0,
  .result = true,
  .forward_backward = true
};

pid_two_wheel pid_right = {
  .val_output = 0.0,
  .setpoint = 0.0,
  .Kp = 0.6,
  .Ki = 5,
  .Kd = 0.0,
  .abs_duration = 0.0,
  .result = true,
  .forward_backward = true
};
double motor_factor = 1.06;

/* declare for PID control */
PID myPID_left(&(pid_left.abs_duration), &(pid_left.val_output), &(pid_left.setpoint), pid_left.Kp, pid_left.Ki, pid_left.Kd, DIRECT);
PID myPID_right(&(pid_right.abs_duration), &(pid_right.val_output), &(pid_right.setpoint), pid_right.Kp, pid_right.Ki, pid_right.Kd, DIRECT);

/* call back function for ros subscriber (check if start) */
void start_cb(const std_msgs::Int32& _msg){
  start = _msg.data;
}

/* initialize node */
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int32> start_cmd("/start_pub", &start_cb);

void setup()
{
  Serial.begin(9600);
  pinMode(RECV_PIN, INPUT);
  t.every(200, time_up);
}

void loop()
{
  if (digitalRead(RECV_PIN) == 0){
    zero_counter++;
  }
  all_counter++;
  
  t.update();
}

void time_up()
{
  Serial.println(zero_counter);

  if (zero_counter > 2700 && zero_counter < 2800){
    Serial.println("detect 600 beacon");
  }else if (zero_counter  > 1400 && zero_counter < 1700){
    Serial.println("detect 1500 beacon");
  }else {
    Serial.println("detect nothing");
  }

  zero_counter = 0;
  all_counter = 0;
}
