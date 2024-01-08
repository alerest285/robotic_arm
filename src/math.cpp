#include <Arduino.h>
#include <String.h>
#include "math.h"

namespace robotic_arm {

double cosDegrees(double x){
   return cos(x * pi / 180); 
}

double cosDegreesDerivative(double x){
   return - (pi / 180) * sin(x * pi / 180); 
}

double sinDegrees(double x){
  return sin(x * pi / 180);
}

double sinDegreesDerivative(double x){
  return (pi / 180) * cos(x * pi / 180);
}

double acosDegrees(double x){
  return acos(x) * 180 / pi;
}

String PlaneCartesianCoordinates::toString(){
    return "x: " + String(x) + ", y: " + String(y);
}

} // namespace robotic_arm
