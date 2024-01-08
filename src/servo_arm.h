#ifndef ROBOTIC_ARM_SERVO_ARM_H
#define ROBOTIC_ARM_SERVO_ARM_H

#include <Servo.h>
#include <String.h>
#include "logging.h"

namespace robotic_arm {

class ServoArm {

  const String _name; 
  const double _length;
  const int _pin;
  const LoggingCallback _logging;
  Servo _servo;
  
  struct MapRange {
    double lower_bound;
    double upper_bound;
    double servo_to_lower_bound;
    double servo_to_upper_bound;
  };
  const MapRange _map_range;
    
  double _current_angle;
  bool _is_current_angle_set;

  double _transformArmAngleToServoAngle(double angle);

  int _nearestIntegerAngle(double angle);

  String _rangeToString();

  public:

    ServoArm(String name, double length, int pin, MapRange map_range, LoggingCallback logging_callback);

    void moveTo(double angle);

    void moveBy(double delta);

    double currentAngle();

    double length();
    
};

} // namespace robotic_arm

#endif // ROBOTIC_ARM_SERVO_ARM_H
