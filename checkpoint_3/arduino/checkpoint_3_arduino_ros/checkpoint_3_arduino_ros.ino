#include <ros.h>
#include <std_msgs/Int32.h>
#include <PID_v1.h>
#include <math.h>

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
  /* initialize the serial port and pin mode */
  Serial.begin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(touch_pin_L, INPUT);
  pinMode(touch_pin_R, INPUT);
  pinMode(touch_pin_M, INPUT);
  pinMode(photo_sensor_pin, INPUT);

  /* initialize PID configuration */
  pid_left.setpoint = 200;
  myPID_left.SetMode(AUTOMATIC);
  myPID_left.SetSampleTime(100);
  pid_right.setpoint = 200;
  myPID_right.SetMode(AUTOMATIC);
  myPID_right.SetSampleTime(100);
  
  /* initialize encoder module */
  EncoderInit();
  
  /* initilaize node */
  nh.initNode();
  nh.subscribe(start_cmd);
}

int counter = 0;
int get_target = 0;
int touch_wall = 0;

void search_LED_target()
{
  if (digitalRead(touch_pin_M) == HIGH){
    get_target = 1;
  }

  if (get_target){
    motor_stop();
  }else{
    val = analogRead(photo_sensor_pin);
    if (val < 100){
      motor_stop();
      delay(1000);
      motor_forward();
      if (digitalRead(touch_pin_M) == HIGH){
        get_target = 1;
        motor_stop();
      }
      delay(1000);
    }else{
      motor_turn_right();
      delay(5);
      counter++;
    }

    if (counter > 800){
      counter = 0;
      motor_forward();
      delay(500);
    }

    if (digitalRead(touch_pin_L) == HIGH){
      touch_left_sensor();
    }else if(digitalRead(touch_pin_R) == HIGH){
      touch_right_sensor();
    }else{
      motor_forward();
    }

    /* calculate control input by PID */
    pid_left.abs_duration = abs(encoder_left.duration);
    pid_left.result = myPID_left.Compute();
    pid_right.abs_duration = abs(encoder_right.duration);
    pid_right.result = myPID_right.Compute();

    /* print motor speed */
    if(pid_left.result)
    {
      encoder_left.duration = 0;
    }
    if(pid_right.result)
    {
      encoder_right.duration = 0;
    }
  }
}

void loop()
{
  if (start){
    if (touch_wall == 0){
      motor_forward_PID();
      if (digitalRead(touch_pin_L) == HIGH || digitalRead(touch_pin_R) == HIGH){
        touch_left_sensor();
        motor_turn_left();
        delay(1000);
        motor_stop();
        delay(500);
        touch_wall++;
      }
    }else if (touch_wall == 1){
      motor_forward();
      delay(2000);
      motor_stop();
      delay(500);
      touch_wall++;
    }else{
      search_LED_target();
    }

    /* calculate control input by PID */
    pid_left.abs_duration = abs(encoder_left.duration);
    pid_left.result = myPID_left.Compute();
    pid_right.abs_duration = abs(encoder_right.duration);
    pid_right.result = myPID_right.Compute();

    /* reset duration of encoder */
    if(pid_left.result)
    {
      encoder_left.duration = 0;
    }
    if(pid_right.result)
    {
      encoder_right.duration = 0;
    }
  }else{
    /* default:static. Or arduino recieve the stopping signal */
    motor_stop();
  }

  nh.spinOnce();
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

/* motor driver function for two wheel */
void motor_forward()  //Motor Forward
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 190);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 200);
}

void motor_backward() //Motor reverse
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 190);
}

void motor_turn_left()  // turn left
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 120);
}

void motor_turn_right()  // turn right
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 160);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

void motor_stop() //Motor stops
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

void touch_left_sensor(){
  motor_stop();
  delay(500);
  motor_backward();
  delay(800);
  motor_stop();
  delay(500);
}

void touch_right_sensor(){
  motor_stop();
  delay(500);
  motor_backward();
  delay(800);
  motor_stop();
  delay(500);
}

void motor_forward_PID()  //Motor Forward
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, pid_left.val_output);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, pid_right.val_output);
}
