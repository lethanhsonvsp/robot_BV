#include "Arduino.h"
#include "Motor.h"
Motor::Motor(int plus, int minus, int pin) {
  pinMode(pin,OUTPUT);
  pinMode(plus,OUTPUT);
  pinMode(minus,OUTPUT);
  Motor::plus = plus;
  Motor::minus = minus;
  Motor::pin = pin;
}
void Motor::rotate(int value) {
  digitalWrite(pin,HIGH);
  if(value>=0){
    int out = map(value, 0, 120, 0, 120);
    analogWrite(plus,out);
    analogWrite(minus,0);
  }else{
    //Max Voltage with 12.5V battery with 12V required
    //(12/12.5)*255 ~=244
    int out = map(value, 0, -120, 0, 120);
    analogWrite(plus,0);
    analogWrite(minus,out);
  }
}
