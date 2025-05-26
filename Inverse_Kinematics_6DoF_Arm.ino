// =================== Inverse_Kinematics_Arduino_6DoF.ino ===================
#include "servo_driver.h"
#include "interface.h"
#include <Arduino.h>

void setup() {
  Serial.begin(9600);
  initServos();

  // Set angle limits for all servos.
  setServoLimits(ShoulderRotate, 0, 180); // Shoulder rotate a.k.a the base.
  setServoLimits(ShoulderTilt, 20, 180);  // Shoulder.
  setServoLimits(Elbow, 10, 170);         // Elbow.
  setServoLimits(WristTilt, 0, 180);      // Wrist tilt.
  setServoLimits(WristRotate, 0, 180);    // Wrist rotate.
  setServoLimits(Gripper, 87, 145);       // Gripper.

  Serial.println("Ready.");
}

void loop() {
  handleSerial();
}