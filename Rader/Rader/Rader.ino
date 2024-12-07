
#include <Servo.h>
#include "NewPing.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiClient.h>

//! Servo -----------------------------------------
#define SERVO_PIN 32
#define SERVO_START 0
#define SERVO_END 100
#define SERVO_WAIT 15

//! Ultrasonic Sensor -----------------------------
#define PING_PIN_1  33 //back
#define PING_PIN_2  25 //front
#define MAX_DISTANCE 107

struct ultrasonic_data {
  int dist1;
  int dist2;
};

int front [101];
int back [101];

//! Wifi ------------------------------------------

// WiFi credentials
const char* router_ssid = "The Creep Next Door";
const char* router_password = "IWon'tTellYouThough!";

// Socket server IP and port
const char* router_ip = "192.168.0.167";
const int router_port = 1234;

//receiving
unsigned int localUdpPort = 1234; // Local port to listen on

WiFiUDP udp;

//! Servo -----------------------------------------
Servo servoMotor;
NewPing sonar1(PING_PIN_1, PING_PIN_1, MAX_DISTANCE); //back
NewPing sonar2(PING_PIN_2, PING_PIN_2, MAX_DISTANCE); //front

void setup() {

  //! Servo -----------------------------------------
  servoMotor.attach(SERVO_PIN);  // attaches the servo on ESP32 pin

  //! Serial ----------------------------------------
  Serial.begin (115200);

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

}

void loop() {

  //! Forward Motion -------------------------------
  // rotates from 0 degrees to n degrees

  /*
  for (int pos = SERVO_START; pos <= SERVO_END; pos += 1) {
    // in steps of 1 degree
    servoMotor.write(flipAngle(pos));
    struct ultrasonic_data mydata = getDistence();

    front[pos] = mydata.dist1;
    back[pos] = mydata.dist2;

    //Serial.print(mydata.dist1); Serial.print(" "); Serial.println(mydata.dist2);
  }

  sendUDP(toJson(front,back));

  delay(SERVO_WAIT);
  */

  //! Backward Motion ------------------------------
  // rotates from n degrees to 0 degrees
  for (int pos = SERVO_END; pos >= SERVO_START; pos -= 1) {
    servoMotor.write(flipAngle(pos));
    struct ultrasonic_data mydata = getDistence();

    front[pos] = mydata.dist1;
    back[pos] = mydata.dist2;

    //Serial.print(mydata.dist1); Serial.print(" "); Serial.println(mydata.dist2);
  }

  sendUDP(toJson(front,back));

  delay(SERVO_WAIT);
}

//? Send data to computer ---------------------------
void sendUDP(String message) {
  udp.beginPacket(router_ip, router_port);
  udp.write((uint8_t *)message.c_str(), message.length());
  udp.endPacket();
}

//? Convert the data to JSON format ----------------
String toJson(int front[], int back[])
{
  JsonDocument doc;
  JsonArray f = doc["f"].to<JsonArray>();
  JsonArray b = doc["b"].to<JsonArray>();

  for (int pos = 0; pos <= 100; pos += 1) {
    f.add(front[pos]);
    b.add(back[pos]);
  }

  String output;
  doc.shrinkToFit();
  serializeJson(doc, output);

  return output;
}

//? Flip the angle to get the correct distance -----
int flipAngle(int originalAngle) {
    if (originalAngle >= 0 && originalAngle <= 180) {
        return 180 - originalAngle;
    } else {
        return 0;
    }
}

//? Get the distance from the ultrasonic sensor -----
struct ultrasonic_data getDistence()
{
  struct ultrasonic_data data;
  data.dist1 = sonar2.ping_cm(MAX_DISTANCE); //front
  data.dist2 = sonar1.ping_cm(MAX_DISTANCE); //back
  return data;
}
