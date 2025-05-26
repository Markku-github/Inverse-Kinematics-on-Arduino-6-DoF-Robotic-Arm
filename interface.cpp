// =================== interface.cpp ===================
#include "interface.h"
#include "ik_solver.h"
#include "servo_driver.h"
#include <Arduino.h>

// Handles incoming serial commands and controls the robotic arm accordingly.
void handleSerial() {
  if (Serial.available()) {
    // Read the incoming serial command from the serial buffer until a newline character is encountered.
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();  // Remove leading and trailing whitespace.

    // Handle movement command in the form: GOTO X=... Y=... Z=... and take optional TILT and ROTATE.
    if (cmd.startsWith("GOTO")) {
      float x = 0, y = 0, z = 0;
      float wTilt = WristTiltDefaultAngle;
      float wRotate = WristRotateDefaultAngle;

      // Find positions of coordinate markers in the command string and possible additional parameters for the wrist.
      int indexX = cmd.indexOf("X=");
      int indexY = cmd.indexOf("Y=");
      int indexZ = cmd.indexOf("Z=");
      int indexTilt = cmd.indexOf("TILT=");
      int indexRotate = cmd.indexOf("ROTATE=");

      // Parse values only if all coordinates are present in the command.
      if (indexX >= 0 && indexY >= 0 && indexZ >= 0) {
        x = cmd.substring(indexX + 2, indexY).toFloat();  // Extract X coordinate.
        y = cmd.substring(indexY + 2, indexZ).toFloat();  // Extract Y coordinate.

        if (indexTilt >= 0 && indexTilt > indexZ) {
          z = cmd.substring(indexZ + 2, indexTilt).toFloat();
          if (indexRotate >= 0 && indexRotate > indexTilt) {
            wTilt = cmd.substring(indexTilt + 5, indexRotate).toInt();
            wRotate = cmd.substring(indexRotate + 7).toInt();
          } else {
            wTilt = cmd.substring(indexTilt + 5).toInt();
          }
        } else if (indexRotate >= 0 && indexRotate > indexZ) {
          z = cmd.substring(indexZ + 2, indexRotate).toFloat();
          wRotate = cmd.substring(indexRotate + 7).toInt();
        } else {
          z = cmd.substring(indexZ + 2).toFloat();
        }

        // Solve inverse kinematics to get joint angles for given position.
        Angles a = solveIK(x, y, z, wTilt, wRotate);

        // Check if the target position is within the arm's reach.
        if (!a.valid) {
          Serial.println("ERROR: IK solution invalid.");
          return;
        }

        // Move servos to calculated joint angles (convert radians to degrees).
        smoothMove(ShoulderRotate, degrees(a.theta0));  // Shoulder rotate a.k.a base rotation.
        smoothMove(ShoulderTilt, degrees(a.theta1));    // Shoulder.
        smoothMove(Elbow, degrees(a.theta2));           // Elbow.  
        smoothMove(WristTilt, degrees(a.theta3));       // Wrist tilt.
        smoothMove(WristRotate, degrees(a.theta4));     // Wrist rotate.

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