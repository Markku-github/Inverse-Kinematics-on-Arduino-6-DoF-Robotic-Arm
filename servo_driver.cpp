// =================== servo_driver.cpp ===================
#include "servo_driver.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int servoAngles[16] = {0};

void initServos() {
  pwm.begin();
  pwm.setPWMFreq(50);
}

// Moves a servo to the given angle.
void moveServo(int channel, int angle) {
  if (channel < 0 || channel >= 16) return;
  angle = constrain(angle, 0, 180);
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