/**
  * This file defines various constants, port numbers, etc. for the code.
  */
#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

//====================
// Numerical Constants.
//====================

const char kNumMotors = 4;

//====================
// Port numbers and information.
// In general, order is: Front, Left, Back, Right.
//====================

char motor_ports[kNumMotors] = {0, 1, 2, 3}; // TODO: Find actual motor ports.
bool motor_inversions[kNumMotors] = {false, false, false, false};

char encoder_ports[kNumMotors * 2] = {0, 1, 2, 3, 4, 5, 6, 7, 8};

#endif  // __CONSTANTS_H__
