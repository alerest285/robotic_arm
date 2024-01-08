#ifndef ROBOTIC_ARM_MATH_H
#define ROBOTIC_ARM_MATH_H

#include <String.h>

namespace robotic_arm {

double cosDegrees(double x);

double cosDegreesDerivative(double x);

double sinDegrees(double x);

double sinDegreesDerivative(double x);

double acosDegrees(double x);

struct PlaneCartesianCoordinates {
  double x;
  double y;
  String toString();
};

} // namespace robotic_arm

#endif // ROBOTIC_ARM_MATH_H
