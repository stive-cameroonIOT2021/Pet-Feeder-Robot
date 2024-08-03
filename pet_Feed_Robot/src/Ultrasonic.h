#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#include <Arduino.h>
class distSensor{
  
   public:
   distSensor(int tPin,int ePin){
   trigPin = tPin;
   echoPin = ePin;

   pinMode(trigPin, OUTPUT);
   pinMode(echoPin, INPUT);

   previousMillis1 = 0, previousMillis2 = 0;
 }

 int calculateDistance(){

  //records time since program start
  unsigned long currentMillis_ = millis();

  long duration;
  digitalWrite(trigPin, LOW); 

  //check if the previous time the method was called is greater than the interval of 2
  if(currentMillis_ - previousMillis1 > 2){
    digitalWrite(trigPin, HIGH);
    previousMillis1 = currentMillis_;
  }

  //check if distance between current time and previous time it was called is greater than interval of 10.
  if(currentMillis_ - previousMillis2 > 10){
      digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  previousMillis2 = currentMillis_;
  }
  return distance;
 }

 private:
     int trigPin;
     int echoPin;
     int distance;
  
     //Will be used in timer
     unsigned long previousMillis1, previousMillis2;
};

#endif
