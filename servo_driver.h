// =================== servo_driver.h ===================
#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

// Initializes the servo controller
void initServos();

// Moves a servo to a specified angle
void moveServo(int channel, int angle);

// Gets the last known angle of a servo
int getServoAngle(int channel);

// Smoothly moves a servo to the target angle over a duration in milliseconds
void smoothMove(int channel, float targetAngle, float durationMs = 1000);

#endif