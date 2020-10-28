const byte encoder0pinA = 2;  // A pin -> the interrupt pin 0 (pint 2)
const byte encoder0pinB = 7;  // B pin -> the digital pin 7

byte encoder0PinALast;
int duration;
boolean Direction;

void setup()
{
  /* initialize the serial port and the encoder module */
  Serial.begin(9600);
  EncoderInit();
}

void loop()
{
  Serial.print("Pulse:");
  Serial.println(duration);
  duration = 0;
  delay(100);
}

void EncoderInit()
{
  /* attach interrupt pin 0 */
  Direction = true;
  pinMode(encoder0pinB, INPUT);
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
