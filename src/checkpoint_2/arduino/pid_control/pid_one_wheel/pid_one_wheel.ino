#include <PID_v1.h>
#define ENA 5 // analog pin for motor PWM
#define IN1 8 // control pin for left motor
#define IN2 9 // control pin for left motor

const byte encoder0pinA = 2;  // A pin -> the interrupt pin 0 (pin 2)
const byte encoder0pinB = 7;  // B pin -> the digital pin 7

/* variables used for encoder */
byte encoder0PinALast;
double duration, abs_duration;
boolean Direction;
boolean result;

/* variables used for PID control */
double val_output;
double Setpoint;
double Kp = 0.6, Ki = 5, Kd = 0;
PID myPID(&abs_duration, &val_output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  /* initialize the serial port and pin mode */
  Serial.begin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  /* initialize PID configuration */
  Setpoint = 120;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(100);
  
  /* initialize encoder module */
  EncoderInit();
}

void loop()
{
  advance();
  abs_duration = abs(duration);
  result = myPID.Compute();
  if(result)
  {
    Serial.print("Pluse: ");
    Serial.println(duration);
    duration = 0;
  }
}

void EncoderInit()
{
  Direction = true;
  pinMode(encoder0pinB,INPUT);
  attachInterrupt(0, wheelSpeed, CHANGE);
}

void wheelSpeed()
{
  int Lstate = digitalRead(encoder0pinA);
  if((encoder0PinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder0pinB);
    if(val == LOW && Direction)
    {
      Direction = false; //Reverse
    }
    else if(val == HIGH && !Direction)
    {
      Direction = true;  //Forward
    }
  }
  encoder0PinALast = Lstate;

  if(!Direction)  duration--;
  else  duration++;
}

void advance()  //Motor Forward
{
  digitalWrite(IN1,LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA,val_output);
}

void back() //Motor reverse
{
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA,val_output);
}

void Stop()//Motor stops
{
  digitalWrite(ENA, LOW);
}
