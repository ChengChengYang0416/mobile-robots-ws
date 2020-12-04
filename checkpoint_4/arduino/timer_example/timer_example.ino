#include "Timer.h"

Timer t;

void setup()
{
  Serial.begin(9600);
  t.every(1000, say_hello);
}

void loop()
{
  t.update();
}

void say_hello()
{
  Serial.println("Hello");
}
