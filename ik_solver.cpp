// =================== ik_solver.cpp ===================
#include "ik_solver.h"
#include "servo_driver.h"
#include <math.h>
#include <Arduino.h>

// Robotic arm segment lengths (in millimeters).
const float L1 = 80.0;  // Base to shoulder (vertical offset). 
const float L2 = 23.0;  // Shoulder rotate point to shoulder tilt point (horizontal offset).
const float L3 = 103.0; // Upper arm.
const float L4 = 145.0; // Forearm.
const float L5 = 180.0; // Wrist + gripper.

// Servo (0-5) angle offsets (to correct physical mounting deviations).
const float s0AngleOffset = 0;              // Base servo a.k.a. shoulder rotation.
const float s1AngleOffset = asin(71.0/L3);  // Shoulder tilt servo. 71.0 is the measured value in millimeters (mm) that the "upper arm" (L3) deviates from the vertical position when the servo is at a 90 degree angle.
const float s2AngleOffset = asin(40.0/L4);  // Elbow servo. 40.0 is the measured value in millimeters (mm) that the "forearm" (L4) deviates from the vertical position when the servo is at a 90 degree angle.
const float s3AngleOffset = 0;              // Wrist tilt servo.
const float s4AngleOffset = 0;              // Wrist rotate servo.

// Helper function: check if angle is within servo limits (in degrees).
bool angleWithinLimits(float angleDeg, int channel) {
  return angleDeg >= servoMinAngles[channel] && angleDeg <= servoMaxAngles[channel];
}

// Inverse Kinematics: solve shoulder and elbow angles in 3D space.
Angles solveIK(float x, float y, float z, float wristTiltDeg, float wristRotateDeg) {
  Angles result;

  // Servo 0: shoulder rotation
  result.theta0 = atan2(y, x) + s0AngleOffset;

  // Calculate the wrist direction vector according to the tilt angle entered by the user.
  float wristTiltRad = radians(wristTiltDeg);
  float wristPlanar = sin(wristTiltRad);
  float wristDx = wristPlanar * cos(result.theta0);
  float wristDy = wristPlanar * sin(result.theta0);
  float wristDz = cos(wristTiltRad);

  // Subtract the wrist length from the target coordinates to get the "forearm end" target point.
  float x2 = x - L5 * wristDx;
  float y2 = y - L5 * wristDy;
  float z2 = z - L5 * wristDz;

  // Calculate shoulder tilt joint position.
  float shoulderTiltX = L2 * cos(result.theta0);
  float shoulderTiltY = L2 * sin(result.theta0);
  float shoulderTiltZ = L1;

  // Target vector relative to shoulder tilt joint.
  float dx = x2 - shoulderTiltX;
  float dy = y2 - shoulderTiltY;
  float dz = z2 - shoulderTiltZ;

  float planarDist = sqrt(dx * dx + dy * dy);
  float totalDist = sqrt(planarDist * planarDist + dz * dz);

  // Check if the target point is reachable.
  if (totalDist > (L3 + L4) || totalDist < fabs(L3 - L4)) {
    result.valid = false;
    Serial.println("===== IK SOLUTION (3D) =====");
    Serial.println("Target point is unreachable (out of range).");
    Serial.print("L3 + L4 Min reach: "); Serial.println(fabs(L3 - L4));
    Serial.print("L3 + L4 max reach: "); Serial.println(L3 + L4);
    Serial.print("Target distance: "); Serial.println(totalDist);
    Serial.println("============================");
    return result;
  }

/*
  Servo 1: shoulder tilt (upper arm).
  The servo is physically installed "wrong way around", so the correction is taken into account in the angle calculation below.
*/
  float angleA = acos((L3 * L3 + totalDist * totalDist - L4 * L4) / (2 * L3 * totalDist));
  float angleB = atan2(dz, planarDist);
  result.theta1 = (M_PI - (angleA + angleB)) + s1AngleOffset;

/*
  Servo 2: elbow (forearm).
  The servo is physically installed "wrong way around", so the correction is taken into account in the angle calculation below.
*/
  float angleElbow = acos((L3 * L3 + L4 * L4 - totalDist * totalDist) / (2 * L3 * L4));
  result.theta2 = (M_PI - angleElbow) + s2AngleOffset;

/*
  Servo 3: wrist tilt.
  Directly according to user input (relative to the global Z axis).
  Wrist tilt must compensate for the tilts of the forearm and upper arm to achieve the target global tilt angle.
*/
  float shoulderTiltAngle = M_PI - result.theta1 + s1AngleOffset;
  float elbowTiltAngle = M_PI - result.theta2 + s2AngleOffset;
  result.theta3 = (M_PI / 180 * 360) - ((M_PI / 180 * 90) + shoulderTiltAngle + elbowTiltAngle);

/*
  Servo 4: Wrist rotate.
  Sets the wrist rotation.
*/
  result.theta4 = radians((wristRotateDeg + s4AngleOffset));

 // Servo limit checks.
  if (!angleWithinLimits(degrees(result.theta0), ShoulderRotate)) {
    Serial.println("===== IK SOLUTION (3D) =====");
    Serial.println("Shoulder rotate angle out of bounds.");
    Serial.print("Shoulder rotate min angle is: "); Serial.println(servoMinAngles[ShoulderRotate]);
    Serial.print("Shoulder rotate max angle is: "); Serial.println(servoMaxAngles[ShoulderRotate]);
    Serial.print("The target angle was: "); Serial.println(degrees(result.theta0));
    Serial.println("============================");
    result.valid = false;
    return result;
  }
  if (!angleWithinLimits(degrees(result.theta1), ShoulderTilt)) {
    Serial.println("===== IK SOLUTION (3D) =====");
    Serial.println("Shoulder tilt angle out of bounds.");
    Serial.print("Shoulder tilt min angle is: "); Serial.println(servoMinAngles[ShoulderTilt]);
    Serial.print("Shoulder tilt max angle is: "); Serial.println(servoMaxAngles[ShoulderTilt]);
    Serial.print("The target angle was: "); Serial.println(degrees(result.theta1));
    Serial.println("============================");
    result.valid = false;
    return result;
  }
  if (!angleWithinLimits(degrees(result.theta2), Elbow)) {
    Serial.println("===== IK SOLUTION (3D) =====");
    Serial.println("Elbow angle out of bounds.");
    Serial.print("Elbow min angle is: "); Serial.println(servoMinAngles[Elbow]);
    Serial.print("Elbow max angle is: "); Serial.println(servoMaxAngles[Elbow]);
    Serial.print("The target angle was: "); Serial.println(degrees(result.theta2));
    Serial.println("============================");
    result.valid = false;
    return result;
  }
  if (!angleWithinLimits(degrees(result.theta3), WristTilt)) {
    Serial.println("===== IK SOLUTION (3D) =====");
    Serial.println("Wrist tilt angle out of bounds.");
    Serial.print("Wrist tilt min angle is: "); Serial.println(servoMinAngles[WristTilt]);
    Serial.print("Wrist tilt max angle is: "); Serial.println(servoMaxAngles[WristTilt]);
    Serial.print("The target angle was: "); Serial.println(degrees(result.theta3));
    Serial.println("============================");
    result.valid = false;
    return result;
  }
  if (!angleWithinLimits(degrees(result.theta4), WristRotate)) {
    Serial.println("===== IK SOLUTION (3D) =====");
    Serial.println("Wrist rotate angle out of bounds.");
    Serial.print("Wrist rotate min angle is: "); Serial.println(servoMinAngles[WristRotate]);
    Serial.print("Wrist rotate max angle is: "); Serial.println(servoMaxAngles[WristRotate]);
    Serial.print("The target angle was: "); Serial.println(degrees(result.theta4));
    Serial.println("============================");
    result.valid = false;
    return result;
  }

  // Debug printout.
  Serial.println("===== IK SOLUTION (3D) =====");
  Serial.print("Target X: "); Serial.print(x);
  Serial.print(" | Y: "); Serial.print(y);
  Serial.print(" | Z: "); Serial.println(z);
  Serial.print("Shoulder rotate (deg): "); Serial.println(degrees(result.theta0));
  Serial.print("Shoulder tilt (deg): "); Serial.println(degrees(M_PI - result.theta1 + s1AngleOffset));
  Serial.print("Elbow tilt (deg): "); Serial.println(degrees(M_PI - result.theta2 + s2AngleOffset));
  Serial.print("Wrist tilt (deg): "); Serial.println(degrees(result.theta3));
  Serial.print("Wrist rotate (deg): "); Serial.println(degrees(result.theta4));
  Serial.println("============================");

  result.valid = true;
  return result;
}