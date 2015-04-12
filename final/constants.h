/**
  * This file defines various constants, port numbers, etc. for the code.
  */
#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

//====================
// Numerical Constants.
//====================

static const char kNumMotors = 4;

//====================
// Port numbers and information.
// In general, order is: Front, Left, Back, Right.
//====================

static char motor_ports[kNumMotors] = {4, 5, 6, 7};
// For inversions, remember that forward/right = non-inverted.
static bool motor_inversions[kNumMotors] = {false, false, false, false};
// TODO: flip wires for down encoders.

static char encoder_ports[kNumMotors * 2] = {2, 22, 3, 23, 18, 24, 19, 25};

#endif  // __CONSTANTS_H__
