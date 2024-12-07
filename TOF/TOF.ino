#include "Adafruit_VL53L0X.h"
#include <Wire.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (!Serial) {
    delay(1);
  }

  Serial.println("Adafruit VL53L0X test.");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  // power
  Serial.println(F("VL53L0X API Single Shot Ranging example\n\n"));

  // Set the timing budget to 20 ms (default is 33 ms)
  lox.setMeasurementTimingBudgetMicroSeconds(20000);
  
  // Set I2C to high speed mode (400 kHz)
  Wire.setClock(400000);
}

void loop() {
  // Start a single shot ranging
  unsigned long startTime = millis();
  lox.startRange();

  // Wait for the ranging to complete
  while (!lox.isRangeComplete()) {
    delayMicroseconds(50); // Very small delay to minimize waiting time
  }

  // Read and print the distance
  unsigned long endTime = millis();
  lox.readRange();
  // Print the time taken for the measurement
  Serial.println(endTime - startTime);
}