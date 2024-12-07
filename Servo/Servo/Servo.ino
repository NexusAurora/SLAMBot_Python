/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-servo-motor
 */

#include <Servo.h>

#define SERVO_PIN 32 // ESP32 pin GPIO26 connected to servo motor
#define SERVO_START 0
#define SERVO_END 100
#define SERVO_WAIT 20

#define SERVO_OFFSET -12
#define SERVO_ERRORED_END 88

Servo servoMotor;

void setup() {
  servoMotor.attach(SERVO_PIN);  // attaches the servo on ESP32 pin
  servoMotor.write(flipAngle(50));
  Serial.begin(9600);
}

int flipAngle(int originalAngle) {
  
  if (originalAngle < SERVO_START) 
  {
    originalAngle = SERVO_START;
  }
  else if(originalAngle > SERVO_END)
  {
    originalAngle = SERVO_END;
  }
  return 180 - map(originalAngle, SERVO_START, SERVO_ERRORED_END, SERVO_START, SERVO_END) + SERVO_OFFSET ;
}

void loop() {

  // rotates from 0 degrees to 180 degrees
  for (int pos = SERVO_START; pos <= SERVO_END; pos += 1) {
    // in steps of 1 degree
    servoMotor.write(flipAngle(pos));
    Serial.println(pos);
    delay(SERVO_WAIT);
  }

  delay(5000);

  // rotates from 180 degrees to 0 degrees
  for (int pos = SERVO_END; pos >= SERVO_START; pos -= 1) {
    servoMotor.write(flipAngle(pos));
    Serial.println(pos);
    delay(SERVO_WAIT);
  }

  delay(5000);
}
