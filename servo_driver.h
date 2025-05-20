// =================== servo_driver.h ===================
#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

// Initializes the PCA9685 servo driver
void setupServos();

// Moves servo at given channel to specified angle (degrees)
void moveServo(int channel, float angleDeg);

#endif