#include "turretpid.h"
#include "Arduino.h"
#include <Servo.h>
// See turretpid.h for a more thorough description.

// Performs simple PID calculation.
// Returns a value which can relatively easily be fed into the arduino
// Servo.write function. The return value will be from -90 to 90 and the write
// function takes a value from 0 to 180.
int TurretPID::Calc() {
  // Calculate Error. Note the analogRead blocks until ADC finishes reading.
  int potval = analogRead(pot_);
  int error = setpoint_ - potval;
  sum_ += (abs(error) > 40) ? 0 : error;
  // Perform actual PID calculation.
  int retval = p_ * error + i_ * sum_ + d_ * (error - prev_error_);
  int max = 40; // maximum allowable throttle (max can range from 0 - 90).
  // Prevent running into stops too hard, especially since there are things that
  // can break near the bottom and top goal positions.
  if (retval > max) retval = max;
  if (retval < -max) retval = -max;
  prev_error_ = error;
//#ifdef DEBUG
  Serial.print("Error:\t");
  Serial.print(error);
  Serial.print("\tOut:\t");
  Serial.println(retval);
//#endif  // DEBUG

  //checks for problems with the potentiometer values
  if (potval > kMaxPot || potval < kMinPot) { //if the potentiometer has been overturned
    return 0;
  }
  if (!digitalRead(max_limit_) &&
      retval > 0) {  // We are running into the upper limit.
    return 0;
  }
  if (!digitalRead(min_limit_) &&
      retval < 0) {  // We are running into the lower limit.
    return 0;
  }
  if (abs(error) < 10) return 0;
  if (stopped_) return 0;
  return retval; //return the pid controlled errors to be used for turret corrections
}
