#include <Servo.h>

// Ports of different flame sensors.
int red = 3;
int box = 11;
int servoport = 11;
const int kStraight = 100;

int angle = -50;

Servo servo;

// Note: the servo is, in practice, limited to -50 to 62.5 degrees
void ServoDeg(int deg) {
  int out = (deg * -1.6 + kStraight);
  servo.write(out);
}

void setup() {
  Serial.begin(115200);
  servo.attach(servoport, 1000, 2000);
  ServoDeg(45);
  delay(5000);
  ServoDeg(0);
  while (true) continue;
}

void loop() {
  ServoDeg(angle);
  delay(300);
  Serial.print(angle);
  Serial.print("\t");
  Serial.print(analogRead(red));
  Serial.print("\t");
  Serial.print(analogRead(box));
  Serial.println();

  angle += 1;
  if (angle > 62) angle = -50;
}
