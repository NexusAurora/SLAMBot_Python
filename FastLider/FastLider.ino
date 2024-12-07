#include <Servo.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "Adafruit_VL53L0X.h"
#include <Wire.h>

//! Servo -----------------------------------------

#define SERVO_PIN 32
#define SERVO_START 0
#define SERVO_END 100
#define SERVO_WAIT 15

#define SERVO_OFFSET -12
#define SERVO_ERRORED_END 88

//! Ultrasonic Sensor -----------------------------

#define MIN_DISTANCE 3
#define MAX_DISTANCE 100

//! LN298N Motor ----------------------------------

#define PIN1 23
#define PIN2 19
#define PIN3 18
#define PIN4 5

#define SPEED 33

const int wait_time = 100;

const int freq = 5000; // Frequency in Hz
const int ledChannel = 3; // PWM channel
const int resolution = 8; // PWM resolution (8-bit)

bool moving = false;

//! TOF Sensor -------------------------------------

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
const unsigned long timeout = 50; // 1 second timeout
int front [101];

//! Wifi ------------------------------------------

// WiFi credentials
const char* router_ssid = "The Creep Next Door";
const char* router_password = "IWon'tTellYouThough!";

// Socket server IP and port
const char* router_ip = "192.168.0.135";
const int router_port = 1234;

//receiving port
unsigned int localUdpPort = 1234; // Local port to listen on

WiFiUDP udp;

// buffer for incoming packets
char incomingPacket[255];  // buffer for incoming packets

//! Structs ---------------------------------------

//move instructions
struct Move
{
  int direction;
  int time;
  int speed;
};

//! Servo -----------------------------------------

Servo servoMotor;
bool forward = true;

void setup() {

  //! Serial ----------------------------------------
  Serial.begin (115200);

  // wait until serial port opens for native USB devices
  while (!Serial) {
    delay(1);
  }
  
  //! TOF Sensor-------------------------------------
  Serial.println("Connecting to TOF");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  
  // Set the timing budget to 20 ms (default is 33 ms)
  lox.setMeasurementTimingBudgetMicroSeconds(20000);
  // Set I2C to high speed mode (400 kHz)
  Wire.setClock(400000);

  //! Servo -----------------------------------------
  servoMotor.attach(SERVO_PIN);  // attaches the servo on ESP32 pin

  //! Wifi ------------------------------------------
  // connect to WiFi network as a station
  WiFi.begin(router_ssid, router_password);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);    // Set WiFi RF power output to highest level
  
  Serial.println("Connecting to Router");
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Begin listening on a specific port (optional for sending)
  udp.begin(localUdpPort);
  Serial.println("UDP setup complete");

  //!Motors -----------------------------------------
  pinMode(PIN1, OUTPUT);
  pinMode(PIN2, OUTPUT);
  pinMode(PIN3, OUTPUT);
  pinMode(PIN4, OUTPUT);

  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(SPEED, ledChannel);

  //! Create tasks ---------------------------------
  // Create tasks and assign them to different cores
  xTaskCreatePinnedToCore(udpTask, "UDP Task", 4096, NULL, 1, NULL, 0); // Core 0 ; parameters are: task function, task name, stack size, parameters, priority, task handle, core
  xTaskCreatePinnedToCore(scanMoveTask, "Scan/Move Task", 4096, NULL, 1, NULL, 1); // Core 1 
}

void loop() {
}

//! Main Functionalities --------------------------

//? Move the robot based on the received data ------
void udpTask(void *pvParameters) {
  for (;;) {
    //! Receive data from computer --------------------
    int packetSize = udp.parsePacket();
    if (packetSize) {
      // Read the incoming packet
      int len = udp.read(incomingPacket, 255);
      if (len > 0) {
        incomingPacket[len] = 0;  // Null-terminate the string
      }

      Serial.println("UDP packet received: ");
      Serial.println(incomingPacket);

      // Deserialize JSON
      Move move = fromJson(incomingPacket);

      // Display the data
      Serial.print("Direction: ");
      Serial.println(move.direction);
      Serial.print("Speed: ");
      Serial.println(move.speed);
      Serial.print("Time: ");
      Serial.println(move.time);

      // 0 means forward
      if (move.direction == 0) {
        moveForward(move.time, move.speed);
      }
      // 1 means backward
      else if (move.direction == 1) {
        moveBackward(move.time, move.speed);
      }
      // 2 means left
      else if (move.direction == 2) {
        moveLeft(move.time, move.speed);
      }
      // 3 means right
      else if (move.direction == 3) {
        moveRight(move.time, move.speed);
      }
      // 4 means stop
      else if (move.direction == 4) {
        stop();
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Add a small delay to prevent task from hogging the CPU
  }
}

//? Scan the environment and send data -------------
void scanMoveTask(void *pvParameters) {

  for (;;) {

    //Only scan if the robot is not moving
    if(!moving)
    {
      if (forward) {
        forward = false;
        //! Forward Motion -------------------------------
        // rotates from 0 degrees to n degrees
        servoMotor.write(flipAngle(SERVO_START));
        delay(wait_time);
        for (int pos = SERVO_START; pos <= SERVO_END; pos += 1) {
          // in steps of 1 degree
          servoMotor.write(flipAngle(pos));
          front[pos] = getDistence();
          // Serial.println(front[pos]);

          // if the robot is moving, stop the servo
          if(moving)
          {
            break;
          }
        }

        // Do not send data if the robot is moving
        if(!moving)
        {
          sendUDP(toJson(front));
          delay(SERVO_WAIT);
        }

      } 
      else 
      {
        forward = true;
        //! Backward Motion ------------------------------
        // rotates from n degrees to 0 degrees
        servoMotor.write(flipAngle(SERVO_END));
        delay(wait_time);
        for (int pos = SERVO_END; pos >= SERVO_START; pos -= 1) {
          servoMotor.write(flipAngle(pos));
          front[pos] = getDistence();
          // Serial.println(front[pos]);

          // if the robot is moving, stop the servo
          if(moving)
          {
            break;
          }
        }

        // Do not send data if the robot is moving
        if(!moving)
        {
          sendUDP(toJson(front));
          delay(SERVO_WAIT);
        }
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);  // Add a small delay to prevent task from hogging the CPU
  }
}

//! Helper Functions ------------------------------

//? Send data to computer ---------------------------
void sendUDP(String message) {
  udp.beginPacket(router_ip, router_port);
  udp.write((uint8_t *)message.c_str(), message.length());
  udp.endPacket();
}

//? Convert the data to JSON format ----------------
String toJson(int front[])
{
  JsonDocument doc;
  JsonArray f = doc["f"].to<JsonArray>();

  for (int pos = 0; pos <= 100; pos += 1) {
    f.add(front[pos]);
  }

  String output;
  doc.shrinkToFit();
  serializeJson(doc, output);

  return output;
}

//? Convert incoming JSON to struct ----------------
Move fromJson(String message)
{
  Move move;
  JsonDocument doc;
  deserializeJson(doc, message);

  move.direction = doc["d"];
  move.speed = doc["s"];
  move.time = doc["t"];

  return move;
}

//? Flip the angle to get the correct distance -----
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

//? Get the distance from the Lider sensor -----
int getDistence()
{
  lox.startRange();
  unsigned long startTime = millis();
  
  // Wait for the ranging to complete
  while (!lox.isRangeComplete()) {
    if (millis() - startTime > timeout) {
      Serial.println("Timeout occurred");
      lox.begin();
      lox.setMeasurementTimingBudgetMicroSeconds(20000);
      return 0;
    }
    delayMicroseconds(50); // Very small delay to minimize waiting time
  }
  int distance = lox.readRange();
  
  if(distance < 30 || distance > 1000)
  {
    return 0;
  }
  else
  {
    return distance;
  }
}

//! Move the robot --------------------------------

void moveForward(int duration, int speed)
{
  moving = true;
  delay(wait_time);
  digitalWrite(PIN1, LOW);
  digitalWrite(PIN2, HIGH);
  digitalWrite(PIN3, LOW);
  digitalWrite(PIN4, HIGH);

  ledcWrite(ledChannel, speed);

  delay(duration);

  stop();
  delay(wait_time);
  moving = false;
}

void moveBackward(int duration, int speed)
{
  moving = true;
  delay(wait_time);
  digitalWrite(PIN1, HIGH);
  digitalWrite(PIN2, LOW);
  digitalWrite(PIN3, HIGH);
  digitalWrite(PIN4, LOW);

  ledcWrite(ledChannel, speed);

  delay(duration);

  stop();
  delay(wait_time);
  moving = false;
}

void moveRight(int duration, int speed)
{
  moving = true;
  delay(wait_time);
  digitalWrite(PIN1, HIGH);
  digitalWrite(PIN2, LOW);
  digitalWrite(PIN3, LOW);
  digitalWrite(PIN4, HIGH);

  ledcWrite(ledChannel, speed);

  delay(duration);

  stop();
  delay(wait_time);
  moving = false;
}

void moveLeft(int duration, int speed)
{
  moving = true;
  delay(wait_time);
  digitalWrite(PIN1, LOW);
  digitalWrite(PIN2, HIGH);
  digitalWrite(PIN3, HIGH);
  digitalWrite(PIN4, LOW);

  ledcWrite(ledChannel, speed);

  delay(duration);

  stop();
  delay(wait_time);
  moving = false;
}

void stop()
{
  digitalWrite(PIN1, LOW);
  digitalWrite(PIN2, LOW);
  digitalWrite(PIN3, LOW);
  digitalWrite(PIN4, LOW);
}