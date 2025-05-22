// =================== interface.h ===================
#ifndef INTERFACE_H
#define INTERFACE_H

// Clamps an angle to be within the valid range for a given servo channel.
float clampAngle(float angle, int channel);

// Handles user input via serial terminal.
void handleSerial();

#endif