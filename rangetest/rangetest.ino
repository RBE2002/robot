#include "Arduino.h"
#include "range.h"

Range *sharp;

void setup() {
  sharp = new Range[4];
  Serial.begin(115200);
  sharp[0].init(0);
  sharp[1].init(1);
  sharp[2].init(2);
  sharp[3].init(3);
}

void loop() {
  Serial.print(sharp[0].Dist());
  Serial.print("\t");
  Serial.print(sharp[1].Dist());
  Serial.print("\t");
  Serial.print(sharp[2].Dist());
  Serial.print("\t");
  Serial.println(sharp[3].Dist());
  delay(100);
}
