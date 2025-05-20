// =================== Inverse_Kinematics_Arduino_6DoF.ino ===================
#include "servo_driver.h"
#include "interface.h"
#include <Arduino.h>

void setup() {
  Serial.begin(9600);
  setupServos();
  Serial.println("Ready.");
}

void loop() {
  handleSerial();
}