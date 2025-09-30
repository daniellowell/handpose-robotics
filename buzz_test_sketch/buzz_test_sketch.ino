#include <Servo.h>
Servo s[6];
const int servoPins[6] = {2,3,4,5,6,7}; // temporary; unused in this "quiet" sketch

void setup() {
  // Don't attach anything yet → no PWM → quiet.
}

void loop() {}
