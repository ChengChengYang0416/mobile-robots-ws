#include <ros.h>
#include <std_msgs/Int32.h>
#include <PID_v1.h>
#include <math.h>

#define IN1 8                 // control pin for left motor
#define IN2 9                 // control pin for left motor
#define IN3 10                // control pin for right motor
#define IN4 11                // control pin for right motor
#define ENA 5                 // analog pin for motor PWM
#define ENB 6                 // analog pin for motor PWN

const byte encoder0pinA = 2;  // A pin -> the interrupt pin 0 (pin 2)
const byte encoder0pinB = 7;  // B pin -> the digital pin 7
const byte encoder1pinA = 3;  // A pin -> the interrupt pin 1 (pin 3)
const byte encoder1pinB = 4;  // B pin -> the digital pin 4

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
  .Kp = 0.2,
  .Ki = 1,
  .Kd = 0.0,
  .abs_duration = 0.0,
  .result = true,
  .forward_backward = true
};

pid_two_wheel pid_right = {
  .val_output = 0.0,
  .setpoint = 0.0,
  .Kp = 0.2,
  .Ki = 1,
  .Kd = 0.0,
  .abs_duration = 0.0,
  .result = true,
  .forward_backward = true
};

/* declare for PID control */
PID myPID_left(&(pid_left.abs_duration), &(pid_left.val_output), &(pid_left.setpoint), pid_left.Kp, pid_left.Ki, pid_left.Kd, DIRECT);
PID myPID_right(&(pid_right.abs_duration), &(pid_right.val_output), &(pid_right.setpoint), pid_right.Kp, pid_right.Ki, pid_right.Kd, DIRECT);

/* call back function for ros subscriber (command of left motor) */
void motor_L_cb(const std_msgs::Int32& _msg){
  if (_msg.data >= 0){
    pid_left.forward_backward = true;
  }else if (_msg.data < 0){
    pid_left.forward_backward = false;
  }
  pid_left.setpoint = fabs((double)_msg.data);
}

/* call back function for ros subscriber (command of right motor) */
void motor_R_cb(const std_msgs::Int32& _msg){
  if (_msg.data >= 0){
    pid_right.forward_backward = true;
  }else if (_msg.data < 0){
    pid_right.forward_backward = false;
  }
  pid_right.setpoint = fabs((double)_msg.data);
}

/* initialize node */
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int32> motor_L_cmd("/motor_L", &motor_L_cb);
ros::Subscriber<std_msgs::Int32> motor_R_cmd("/motor_R", &motor_R_cb);

void setup()
{
  /* initialize the serial port and pin mode */
  Serial.begin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  /* initialize PID configuration */
  pid_left.setpoint = 120;
  myPID_left.SetMode(AUTOMATIC);
  myPID_left.SetSampleTime(100);
  pid_right.setpoint = 120;
  myPID_right.SetMode(AUTOMATIC);
  myPID_right.SetSampleTime(100);
  
  /* initialize encoder module */
  EncoderInit();

  /* initilaize node */
  nh.initNode();
  nh.subscribe(motor_L_cmd);
  nh.subscribe(motor_R_cmd);
}

void loop()
{
  /* actuate the motors */
  if (pid_left.forward_backward == true){
    forward_left();
  }else if(pid_left.forward_backward == false){
    backward_left();
  }
  if (pid_right.forward_backward == true){
    forward_right();
  }else if (pid_right.forward_backward == false){
    backward_right();
  }

  /* calculate control input by PID */
  pid_left.abs_duration = abs(encoder_left.duration);
  pid_left.result = myPID_left.Compute();
  pid_right.abs_duration = abs(encoder_right.duration);
  pid_right.result = myPID_right.Compute();

  /* print motor speed */
  if(pid_left.result)
  {
    Serial.print("Pulse left and right : ");
    Serial.print(encoder_left.duration);
    encoder_left.duration = 0;
  }
  if(pid_right.result)
  {
    Serial.print(" ");
    Serial.println(encoder_right.duration);
    encoder_right.duration = 0;
  }

  nh.spinOnce();
  delay(10);
}

void EncoderInit()
{
  /* initialize encoder module */
  encoder_left.Direction = true;
  encoder_right.Direction = true;
  pinMode(encoder0pinB, INPUT);
  pinMode(encoder1pinB, INPUT);
  attachInterrupt(0, wheelSpeed0, CHANGE);
  attachInterrupt(1, wheelSpeed1, CHANGE);
}

/* interrupt service routine for interrupt 0 */
void wheelSpeed0()
{
  int Lstate = digitalRead(encoder0pinA);
  if((encoder_left.encoderPinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder0pinB);
    if(val == LOW && encoder_left.Direction)
    {
      encoder_left.Direction = false; //Reverse
    }
    else if(val == HIGH && !(encoder_left.Direction))
    {
      encoder_left.Direction = true;  //Forward
    }
  }
  encoder_left.encoderPinALast = Lstate;

  if(!encoder_left.Direction)  encoder_left.duration--;
  else  encoder_left.duration++;
}

/* interrupt service routine for interrupt 1 */
void wheelSpeed1()
{
  int Lstate = digitalRead(encoder1pinA);
  if((encoder_right.encoderPinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder1pinB);
    if(val == LOW && encoder_right.Direction)
    {
      encoder_right.Direction = false; //Reverse
    }
    else if(val == HIGH && !(encoder_right.Direction))
    {
      encoder_right.Direction = true;  //Forward
    }
  }
  encoder_right.encoderPinALast = Lstate;

  if(!encoder_right.Direction)  encoder_right.duration--;
  else  encoder_right.duration++;
}

/* motor driver function for left wheel */
void forward_left()  //Motor Forward
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, pid_left.val_output);
}

void backward_left() //Motor reverse
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, pid_left.val_output);
}

void stop_left()//Motor stops
{
  digitalWrite(ENA, LOW);
}

/* motor driver function for right wheel */
void forward_right()  //Motor Forward
{
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, pid_right.val_output);
}

void backward_right() //Motor reverse
{
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, pid_right.val_output);
}

void stop_right()//Motor stops
{
  digitalWrite(ENB, LOW);
}
