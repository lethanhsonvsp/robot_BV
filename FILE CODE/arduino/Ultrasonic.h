#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#include <Arduino.h>

class Ultrasonic {
public:
  Ultrasonic(int trigPin, int echoPin);
  long getDistance();
private:
  int trigPin; int echoPin;
};
#endif // ULTRASONIC_H
