#include "Arduino.h"
#include "range.h"

Range *sharp;

void setup() {
  Serial.begin(115200);
  sharp = new Range(0, Range::kSharp);
}

void loop() {
  sharp->Update();
  Serial.println(sharp->Dist());
  delay(100);
}
