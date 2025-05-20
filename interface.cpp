// =================== interface.cpp ===================
#include "interface.h"
#include "ik_solver.h"
#include "servo_driver.h"
#include <Arduino.h>

void handleSerial() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("GOTO")) {
      float x = 0, y = 0, z = 0;
      int indexX = cmd.indexOf("X=");
      int indexY = cmd.indexOf("Y=");
      int indexZ = cmd.indexOf("Z=");

      if (indexX >= 0 && indexY >= 0 && indexZ >= 0) {
        x = cmd.substring(indexX + 2, indexY).toFloat();
        y = cmd.substring(indexY + 2, indexZ).toFloat();
        z = cmd.substring(indexZ + 2).toFloat();

        Angles a = solveIK(x, y, z);

        if (!a.valid) {
          Serial.println("ERROR: Unreachable point.");
          return;
        }

        moveServo(0, a.theta0 * 180.0 / M_PI);
        moveServo(1, a.theta1 * 180.0 / M_PI);
        moveServo(2, a.theta2 * 180.0 / M_PI);
        Serial.println("Moved.");
      } else {
        Serial.println("ERROR: Invalid GOTO format.");
      }
    } else if (cmd == "GRIP OPEN") {
      moveServo(5, 0);
    } else if (cmd == "GRIP CLOSE") {
      moveServo(5, 90);
    } else {
      Serial.println("Unknown command.");
    }
  }
}
