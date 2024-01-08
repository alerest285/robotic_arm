#ifndef ROBOTIC_ARM_MATH_H
#define ROBOTIC_ARM_MATH_H

#include <String.h>

namespace robotic_arm {

constexpr double pi = 3.1415926535897932384626433;

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
