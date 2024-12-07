
/*
#define PIN1 19
#define PIN2 18
*/

#define PIN1 23
#define PIN2 19
#define PIN3 18
#define PIN4 5

#define SPEED 33

const int freq = 5000; // Frequency in Hz
const int ledChannel = 0; // PWM channel
const int resolution = 8; // PWM resolution (8-bit)

void setup() {
  pinMode(PIN1, OUTPUT);
  pinMode(PIN2, OUTPUT);
  pinMode(PIN3, OUTPUT);
  pinMode(PIN4, OUTPUT);

  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(SPEED, ledChannel);

  ledcWrite(ledChannel, 100);
}

void loop() {
  digitalWrite(PIN1, HIGH);
  digitalWrite(PIN2, LOW);
  digitalWrite(PIN3, HIGH);
  digitalWrite(PIN4, LOW);
}
