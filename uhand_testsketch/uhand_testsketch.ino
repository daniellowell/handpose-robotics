#include <Servo.h>

Servo s[6];

// Order: [thumb, index, middle, ring, pinky, swivel]
const int servoPins[6] = {7, 6, 5, 4, 3, 2};

// Per-channel inversion (1 = invert, 0 = normal)
const uint8_t invert[6] = {1, 0, 0, 0, 0, 0};  // thumb only

// Conservative per-finger limits (tune after testing)
int angMin[6] = {70, 70, 70, 75, 70, 80};
int angMax[6] = {110,110,110,105,110,100};

int clampAndInvert(int i, int a) {
  a = constrain(a, 0, 180);
  if (invert[i]) a = 180 - a;             // invert direction if needed
  return constrain(a, angMin[i], angMax[i]);
}

void setup() {
  Serial.begin(9600);
  // Attach and go to neutral (clamped)
  for (int i=0; i<6; i++) {
    s[i].attach(servoPins[i]);
    s[i].write(clampAndInvert(i, 90));
  }
  Serial.println("uHand Option A ready. Send 'a,b,c,d,e,f' (0..180). Order: thumb,index,middle,ring,pinky,swivel");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    int a[6] = {90,90,90,90,90,90};
    int n = sscanf(line.c_str(), "%d,%d,%d,%d,%d,%d", &a[0],&a[1],&a[2],&a[3],&a[4],&a[5]);
    if (n >= 1) {
      for (int i=0; i<6; i++) {
        s[i].write(clampAndInvert(i, a[i]));
      }
    }
  }
}
