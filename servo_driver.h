// =================== servo_driver.h ===================
#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

constexpr int ShoulderRotate = 0; // Servo channel for shoulder rotate.
constexpr int ShoulderTilt = 1;   // Servo channel for shoulder tilt.
constexpr int Elbow = 2;          // Servo channel for elbow.
constexpr int WristTilt = 3;      // Servo channel for wrist tilt.
constexpr int WristRotate = 4;    // Servo channel for wrist rotate.
constexpr int Gripper = 5;        // Servo channel for gripper.

constexpr float WristTiltDefaultAngle = 90;    // Wrist tilt default angle.
constexpr float WristRotateDefaultAngle = 90;  // Wrist rotate default angle.

// Expose angle limits to other files
extern int servoMinAngles[16];
extern int servoMaxAngles[16];

// Initializes the servo controller.
void initServos();

// Sets min and max angles for a specific servo.
void setServoLimits(int channel, int minAngle, int maxAngle);

// Moves a servo to a specified angle.
void moveServo(int channel, int angle);

// Gets the last known angle of a servo.
int getServoAngle(int channel);

// Smoothly moves a servo to the target angle over a duration in milliseconds.
void smoothMove(int channel, float targetAngle, float durationMs = 2000);

#endif