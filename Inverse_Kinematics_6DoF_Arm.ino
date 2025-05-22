// =================== Inverse_Kinematics_Arduino_6DoF.ino ===================
#include "servo_driver.h"
#include "interface.h"
#include <Arduino.h>

void setup() {
  Serial.begin(9600);
  initServos();

  // Set angle limits for all servos.
  setServoLimits(0, 0, 180);  // Base.
  setServoLimits(1, 45, 180); // Shoulder.
  setServoLimits(2, 10, 170); // Elbow.
  setServoLimits(5, 87, 145); // Gripper.

  Serial.println("Ready.");
}

void loop() {
  handleSerial();
}