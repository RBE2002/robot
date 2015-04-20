/**
  * This file defines various constants, port numbers, etc. for the code.
  */
#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

//====================
// Numerical Constants.
//====================

static const int kNumMotors = 4;

//====================
// Port numbers and information.
// In general, order is: Front, Left, Back, Right.
//====================

// Drivetrain stuff:
static int motor_ports[kNumMotors] = {4, 5, 6, 7};
// For inversions, remember that forward/right = non-inverted.
static bool motor_inversions[kNumMotors] = {false, false, true, true};
// TODO: flip wires for down encoders.
//static int encoder_ports[kNumMotors * 2] = {2, 22, 3, 23, 18, 24, 19, 25};
static int encoder_ports[kNumMotors * 2] = {2, 2, 3, 3, 18, 18, 19, 19};

// Turret
static int turret_motor = 8;
static int turret_pot = 9;

// Fan & Tilt
static int fan_port = 29; // digital
static int tilt_port = 11; // digital

// Range sensors
// Analog Ports.
static int range_ports[kNumMotors] = {1, 2, 3, 0};

// Cliff Detector ports.
static int cliff_ports[kNumMotors] = {5, 6, 7, 8};

// Flame sensors
static int red_port = 4;
static int black_port = 11;

// Debug/Ancillary
static int start_button = 28;

#endif  // __CONSTANTS_H__
