// =================== ik_solver.cpp ===================
#include "ik_solver.h"
#include <math.h>
#include <Arduino.h>


// Lengths of each segment of the robotic arm (in mm).
const float L1 = 91.0;  // Base to shoulder.
const float L2 = 105.0;  // Shoulder to elbow.
const float L3 = 143.0;  // Elbow to wrist.

// Solves inverse kinematics for a 3DOF planar robotic arm.
Angles solveIK(float x, float y, float z) {
  Angles result;

  // Calculate the base rotation angle in the XY plane.
  result.theta0 = atan2(y, x);

  // Convert the target point into a 2D plane (arm projection).
  float planarDist = sqrt(x * x + y * y);      // Distance in the XY plane.
  float zOffset = z - L1;                      // Height relative to shoulder.
  float d = sqrt(planarDist * planarDist + zOffset * zOffset);  // Total distance to target from shoulder joint.

  // Check if the target point is within reach based on arm geometry.
  if (d > (L2 + L3) || d < fabs(L2 - L3)) {
    result.valid = false;  // Target is unreachable.
    return result;
  }

  // Apply the Law of Cosines to determine angle between L2 and target point.
  float a = acos((L2 * L2 + d * d - L3 * L3) / (2 * L2 * d));
  float b = atan2(zOffset, planarDist);  // Angle from shoulder to target point.
  result.theta1 = b + a;  // Shoulder angle.

  // Apply the Law of Cosines again to determine elbow angle.
  float c = acos((L2 * L2 + L3 * L3 - d * d) / (2 * L2 * L3));
  result.theta2 = M_PI - c;  // Elbow angle (inward bend).

  result.valid = true;  // Mark result as valid.
  Serial.print("θ0: "); Serial.println(result.theta0 * 180 / M_PI);
  Serial.print("θ1: "); Serial.println(result.theta1 * 180 / M_PI);
  Serial.print("θ2: "); Serial.println(result.theta2 * 180 / M_PI);
  return result;
}