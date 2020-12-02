#include "Timer.h"
#include <IRremote.h>

Timer t;
const int RECV_PIN = A2;
IRrecv irrecv(RECV_PIN);
int zero_counter = 0;
int all_counter = 0;

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
