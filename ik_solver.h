// =================== ik_solver.h ===================
#ifndef IK_SOLVER_H
#define IK_SOLVER_H

// Struct to store resulting joint angles
struct Angles {
  float theta0;  // Base rotation
  float theta1;  // Shoulder angle
  float theta2;  // Elbow angle
  bool valid;    // Solution validity
};

// Function to solve inverse kinematics for given X, Y, Z
Angles solveIK(float x, float y, float z);

#endif