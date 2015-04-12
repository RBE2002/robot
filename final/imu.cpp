#include "loop.h"
#include "imu.h"

IMU::IMU()
    : Loop(1e4 /*1e4us=>100Hz*/),
      gyro_zero_(0.0),
      est_rate_(0.0),
      est_rate_weight_(0),
      est_angle_(0.0),
      est_angle_weight_(0) {
  Wire.begin();

  // Initialize the compass.
  compass_.init();
  compass_.enableDefault();
  // CTRL6 controls the resolution.
  compass_.writeReg(LSM303::CTRL6, k2Gauss);
  // Turn off accelerometer.
  char AODR = 0x6 << 4; // 100Hz accel data selection.
  char AZEN = 0x1 << 2;
  char AYEN = 0x1 << 1;
  char AXEN = 0x1;
  char CTRL1 = AODR | AZEN | AYEN | AXEN;
  compass_.writeReg(LSM303::CTRL1, CTRL1);
  // Update compass at 100Hz.
  char TEMP_EN = 0x00;  // Default (Disabled); 0x80 to enable. 1 bit.
  char M_RES = 0x60;    // Default (High Res); 0x00 for low. 2 bits.
  char M_ODR = 0x14;    // 100Hz (not default). 3 bits.
  char data = TEMP_EN | M_RES | M_ODR;
  compass_.writeReg(LSM303::CTRL5, data);
  compass_.m_min = (LSM303::vector<int16_t>){-1382, -2367, -4469};
  compass_.m_max = (LSM303::vector<int16_t>){2695, 1600, 3258};

  // Initialize the gyro. It is already initialized to the lowest dps option.
  gyro_.init();
  gyro_.enableDefault();

}

void IMU::CalibrateGyro() {
  gyro_zero_ = 0;
  int iterations = 100;
  for (int i = 0; i < iterations; i++) {
    gyro_.read();
    gyro_zero_ += get_gyro_vel();
    delay(50);
  }
  gyro_zero_ /= iterations;
}

// Called at 100Hz by the Loop stuff.
void IMU::Run() {
  time_ = micros();
  gyro_.read();
  compass_.read();

  UpdateCompass();
  Filter();

  last_time_ = time_;
}

bool IMU::RejectCompass() {
  return true;
  // If any of various conditions occur, then it will reject.

  double compass_rate = get_compass_rate();
  const double kRateCutoff = 20.0;

  // Check if compass rate seriously mis-matches estimated rate.
  if (abs(est_rate_ - compass_rate_) > kRateCutoff && est_rate_weight_) return true;

  // Check if compass rate matches the gyro rate.
  if (abs(get_gyro_vel() - compass_rate_) > kRateCutoff) return true;

  // Check if there is an excessive magnetic field.
  // This is not a perfect test, but is just meant as an order-of-magnitude thing.
  if (abs(compass_.m.x) + abs(compass_.m.y) + abs(compass_.m.z) >
      kMaxRawCompass)
    return true;

  return false;
}

bool IMU::UpdateCompass() {
  if (compass_.heading() == last_compass_heading_) return false;

  compass_rate_ =
      (compass_.heading() - last_compass_heading_) / (time_ - last_time_) * 1e6;

  last_compass_heading_ = compass_.heading();
  return true;
}

void IMU::Filter() {
  // For now, just use simplistic averaging.
  rate_ = (get_gyro_vel() * kGyroRateWeight +
           est_rate_ * est_rate_weight_) /
          (kGyroRateWeight + est_rate_weight_);

  angle_ = (rate_ * (time_ - last_time_) * kPreviousAngleWeight +
            est_angle_ * est_rate_weight_) /
           (kPreviousAngleWeight + est_rate_weight_);
}
