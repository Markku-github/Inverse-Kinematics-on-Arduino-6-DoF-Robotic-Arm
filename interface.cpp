// =================== interface.cpp ===================
#include "interface.h"
#include "ik_solver.h"
#include "servo_driver.h"
#include <Arduino.h>

const int Gripper = 5;  // Gripper channel number.

// Handles incoming serial commands and controls the robotic arm accordingly.
void handleSerial() {
  if (Serial.available()) {
    // Read the incoming serial command from the serial buffer until a newline character is encountered.
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();  // Remove leading and trailing whitespace.

    // Handle movement command in the form: GOTO X=... Y=... Z=...
    if (cmd.startsWith("GOTO")) {
      float x = 0, y = 0, z = 0;
      // Find positions of coordinate markers in the command string.
      int indexX = cmd.indexOf("X=");
      int indexY = cmd.indexOf("Y=");
      int indexZ = cmd.indexOf("Z=");

      // Parse values only if all coordinates are present in the command.
      if (indexX >= 0 && indexY >= 0 && indexZ >= 0) {
        x = cmd.substring(indexX + 2, indexY).toFloat();  // Extract X coordinate.
        y = cmd.substring(indexY + 2, indexZ).toFloat();  // Extract Y coordinate.
        z = cmd.substring(indexZ + 2).toFloat();          // Extract Z coordinate.

        // Solve inverse kinematics to get joint angles for given position.
        Angles a = solveIK(x, y, z);

        // Check if the target position is within the arm's reach.
        if (!a.valid) {
          Serial.println("ERROR: Unreachable point.");
          return;
        }

        // Move servos to calculated joint angles (convert radians to degrees).
        smoothMove(0, a.theta0 * 180.0 / M_PI);  // Base rotation.
        smoothMove(1, a.theta1 * 180.0 / M_PI);  // Shoulder.
        smoothMove(2, a.theta2 * 180.0 / M_PI);  // Elbow.
        Serial.println("Moved.");
      } else {
        Serial.println("ERROR: Invalid GOTO format.");
      }
    } 
    // Handle gripper open command.
    else if (cmd == "GRIP OPEN") {
      smoothMove(Gripper, 87);  // Open the gripper.
      Serial.println("Opened.");
    } 
    // Handle gripper close command.
    else if (cmd == "GRIP CLOSE") {
      smoothMove(Gripper, 145);  // Close the gripper.
      Serial.println("Closed.");
    } 
    // Handle unknown or malformed commands.
    else {
      Serial.println("Unknown command.");
    }
  }
}