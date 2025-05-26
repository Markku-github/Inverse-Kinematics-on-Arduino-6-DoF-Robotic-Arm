// =================== ik_solver.h ===================
#ifndef IK_SOLVER_H
#define IK_SOLVER_H

bool angleWithinLimits(float angle, int channel);

// Struct to store resulting joint angles.
struct Angles {
  float theta0; // Base rotation.
  float theta1; // Shoulder angle.
  float theta2; // Elbow angle.
  float theta3; // Wrist tilt.
  float theta4; // Wrist rotate.
  bool valid;   // Solution validity.
};

/*
  Function to solve inverse kinematics for given X, Y, Z.
  Extended with wrist tilt.
*/
Angles solveIK(float x, float y, float z, float wristTiltDeg, float wristRotateDeg);

#endif