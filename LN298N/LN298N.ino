#include <Arduino.h>

/*
#define SPEED1 15
#define SPEED2 6
*/
#define IN1 23
#define IN2 19
#define IN3 18
#define IN4 5

void setup() {
  /*
  pinMode(SPEED1, OUTPUT);
  pinMode(SPEED2, OUTPUT);
  */
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  moveForward(255);
}

void loop() {

}

void moveForward(int speed) {
  /*
  digitalWrite(SPEED1, HIGH);
  digitalWrite(SPEED2, HIGH);
  */
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

/*
void moveBackward(int speed) {
  analogWrite(SPEED1, speed);
  analogWrite(SPEED2, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft(int speed) {
  analogWrite(SPEED1, speed);
  analogWrite(SPEED2, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight(int speed) {
  analogWrite(SPEED1, speed);
  analogWrite(SPEED2, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stop() {
  analogWrite(SPEED1, 0);
  analogWrite(SPEED2, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
*/

