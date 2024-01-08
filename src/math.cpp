#include <Arduino.h>
#include <String.h>
#include "math.h"

namespace robotic_arm {

double cosDegrees(double x){
   return cos(x * PI / 180); 
}

double cosDegreesDerivative(double x){
   return - (PI / 180) * sin(x * PI / 180); 
}

double sinDegrees(double x){
  return sin(x * PI / 180);
}

double sinDegreesDerivative(double x){
  return (PI / 180) * cos(x * PI / 180);
}

double acosDegrees(double x){
  return acos(x) * 180 / PI;
}

String PlaneCartesianCoordinates::toString(){
    return "x: " + String(x) + ", y: " + String(y);
}

} // namespace robotic_arm
