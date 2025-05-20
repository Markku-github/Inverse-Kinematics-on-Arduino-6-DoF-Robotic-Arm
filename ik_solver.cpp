// =================== ik_solver.cpp ===================
#include "ik_solver.h"
#include <math.h>

// Define arm segment lengths (in mm)
#define L1 105.0  // Upper arm length (in mm)
#define L2 147.0  // Forearm length (in mm)

Angles solveIK(float x, float y, float z) {
  Angles result;

  float r = sqrt(x*x + y*y);     // Horizontal distance from base
  result.theta0 = atan2(y, x);   // Base rotation angle

  float d = sqrt(r*r + z*z);     // Total straight-line distance

  // Check if point is reachable
  if (d > (L1 + L2)) {
    result.valid = false;
    return result;
  }

  // Using cosine law and triangle geometry
  float a = acos((L1*L1 + d*d - L2*L2) / (2 * L1 * d));
  float b = atan2(z, r);
  result.theta1 = b + a;

  result.theta2 = acos((L1*L1 + L2*L2 - d*d) / (2 * L1 * L2));
  result.theta2 = M_PI - result.theta2; // Elbow down configuration

  result.valid = true;
  return result;
}
