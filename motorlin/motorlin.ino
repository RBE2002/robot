#include <Encoder.h>
#include <Servo.h>
#include "constants.h"

Encoder *fenc, *lenc, *benc, *renc;
Servo motors[kNumMotors];

long last_enc[kNumMotors];
int power = 180;

void setup() {
  Serial.begin(115200);
  fenc = new Encoder(encoder_ports[0], encoder_ports[1]);
  lenc = new Encoder(encoder_ports[2], encoder_ports[3]);
  benc = new Encoder(encoder_ports[4], encoder_ports[5]);
  renc = new Encoder(encoder_ports[6], encoder_ports[7]);

  for (int i = 0; i < kNumMotors; i++) {
    motors[i].attach(motor_ports[i], 1000, 2000);
    last_enc[i] = 0;
  }
}

void loop() {
  for (int i = 0; i < kNumMotors; i++) {
    motors[i].write(power);
  }
  delay(1000);
  long start[4];
  start[0] = fenc->read();
  start[1] = lenc->read();
  start[2] = benc->read();
  start[3] = renc->read();
  delay(500);
  long after[4];
  after[0] = fenc->read();
  after[1] = lenc->read();
  after[2] = benc->read();
  after[3] = renc->read();

  long vel[4];
  Serial.print(power);
  Serial.print("\t");
  for (int i = 0; i < kNumMotors; i++) {
    vel[i] = (after[i] - start[i]) * 2;
    Serial.print(vel[i]);
    Serial.print("\t");
  }
  Serial.println();

  power -= 2;
  if (power < 0) exit(0);
}
