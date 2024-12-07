
#include "NewPing.h"

#define PING_PIN  33 //25 - 33
#define MAX_DISTANCE 400

NewPing sonar(PING_PIN, PING_PIN, MAX_DISTANCE);

void setup()
{
  Serial.begin (115200);
}
 
void loop()
{
  Serial.println(getDistence());
}

float getDistence()
{
  return sonar.ping_cm();
}