// =================== servo_driver.cpp ===================
#include "servo_driver.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const int MinAngle = 45;  // Gripper channel number.
const int MaxAngle = 170;  // Gripper channel number.

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int servoAngles[16] = {0};
int servoMinAngles[16];
int servoMaxAngles[16];

void initServos() {
  pwm.begin();
  pwm.setPWMFreq(50);

  // Default limits for all servos: 0–180 degrees.
  for (int i = 0; i < 16; i++) {
    servoMinAngles[i] = 0;
    servoMaxAngles[i] = 180;
  }
}

void setServoLimits(int channel, int minAngle, int maxAngle) {
  if (channel < 0 || channel >= 16) return;
  servoMinAngles[channel] = constrain(minAngle, 0, 180);
  servoMaxAngles[channel] = constrain(maxAngle, 0, 180);
}

// Moves a servo to the given angle within its defined limits.
void moveServo(int channel, int angle) {
  if (channel < 0 || channel >= 16) return;
  angle = constrain(angle, servoMinAngles[channel], servoMaxAngles[channel]);
  int pulse = map(angle, 0, 180, 102, 512);
  pwm.setPWM(channel, 0, pulse);
  servoAngles[channel] = angle;
}

// Returns the last known angle of the servo.
int getServoAngle(int channel) {
  if (channel < 0 || channel >= 16) return 0;
  return servoAngles[channel];
}

/*
  Smoothly moves a servo from its current angle to the target angle in small steps.
  durationMs defines how long the movement should take in total (default: 1000ms).
  Step count and delay between steps are computed to make the motion fluid.
*/
void smoothMove(int channel, float targetAngle, float durationMs) {
  int currentAngle = getServoAngle(channel);
  int steps = 100;
  float stepDelay = durationMs / steps;
  float stepSize = (targetAngle - currentAngle) / steps;

  if (targetAngle != currentAngle) {
    for (int i = 1; i <= steps; i++) {
      moveServo(channel, currentAngle + stepSize * i);
      delay(stepDelay);
    }
  }
}