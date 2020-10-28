const byte encoder0pinA = 2;  // A pin -> the interrupt pin 0 (pin 2)
const byte encoder0pinB = 7;  // B pin -> the digital pin 7
const byte encoder1pinA = 3;  // A pin -> the interrupt pin 1 (pin 3)
const byte encoder1pinB = 4;  // B pin -> the digital pin 4

typedef struct
{
  byte encoderPinALast;
  int duration;
  boolean Direction;
}encoder_two_wheel;

encoder_two_wheel motor_left;
encoder_two_wheel motor_right;

void setup()
{
  /* initialize the serial port and the encoder module */
  Serial.begin(9600);
  EncoderInit();
}

void loop()
{
  Serial.print("pulse left : ");
  Serial.print(motor_left.duration);
  Serial.print(", pulse right : ");
  Serial.println(motor_right.duration);
  motor_left.duration = 0;
  motor_right.duration = 0;
  delay(100);
}

void EncoderInit()
{
  /* attach interrupt pin 0 */
  motor_left.Direction = true;
  motor_right.Direction = true;
  pinMode(encoder0pinB, INPUT);
  pinMode(encoder1pinB, INPUT);
  attachInterrupt(0, wheelSpeed0, CHANGE);
  attachInterrupt(1, wheelSpeed1, CHANGE);
}

void wheelSpeed0()
{
  int Lstate = digitalRead(encoder0pinA);
  if((motor_left.encoderPinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder0pinB);
    if(val == LOW && motor_left.Direction)
    {
      motor_left.Direction = false; //Reverse
    }
    else if(val == HIGH && !(motor_left.Direction))
    {
      motor_left.Direction = true;  //Forward
    }
  }
  motor_left.encoderPinALast = Lstate;

  if(!motor_left.Direction)  motor_left.duration--;
  else  motor_left.duration++;
}

void wheelSpeed1()
{
  int Lstate = digitalRead(encoder1pinA);
  if((motor_right.encoderPinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder1pinB);
    if(val == LOW && motor_right.Direction)
    {
      motor_right.Direction = false; //Reverse
    }
    else if(val == HIGH && !(motor_right.Direction))
    {
      motor_right.Direction = true;  //Forward
    }
  }
  motor_right.encoderPinALast = Lstate;

  if(!motor_right.Direction)  motor_right.duration--;
  else  motor_right.duration++;
}
