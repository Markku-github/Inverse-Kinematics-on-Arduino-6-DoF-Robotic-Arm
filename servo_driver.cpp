// =================== servo_driver.cpp ===================
#include "servo_driver.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm_board = Adafruit_PWMServoDriver(0x40); // called this way, it uses the default address 0x40  

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_MIN 150 // Min pulse length (0 deg)
#define SERVO_MAX 600 // Max pulse length (180 deg)
#define ANGLE_MIN 0   // Min servo angle
#define ANGLE_MAX 180 // Max servo angle

// Converts angle in degrees to PWM pulse length
int angleToPulse(float angle) {
  return map((int)angle, ANGLE_MIN, ANGLE_MAX, SERVO_MIN, SERVO_MAX);
}

// Initializes the PCA9685 board
void setupServos() {
  pwm_board.begin();
  pwm_board.setPWMFreq(50);  // Set frequency to 50Hz for servos
}

// Moves the specified servo to the target angle
void moveServo(int channel, float angleDeg) {
  int pulse = angleToPulse(angleDeg);
  pwm_board.setPWM(channel, 0, pulse);
}
