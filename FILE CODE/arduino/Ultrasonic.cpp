#include "ultrasonic.h"
Ultrasonic::Ultrasonic(int trigPin, int echoPin) {
  this->trigPin = trigPin;
  this->echoPin = echoPin;
  pinMode(trigPin, OUTPUT);
 pinMode(echoPin, INPUT);
}

long Ultrasonic::getDistance() {
  digitalWrite(trigPin, LOW);delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration / 29.1 / 2;
}
