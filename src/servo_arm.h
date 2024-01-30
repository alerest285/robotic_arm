#ifndef ROBOTIC_ARM_SERVO_ARM_H
#define ROBOTIC_ARM_SERVO_ARM_H

#include <Servo.h>
#include <String.h>
#include "logging.h"

namespace robotic_arm {

class ServoArm {

  const String _name; 
  const double _length;
  const LoggingCallback _logging;
  Servo* _servo;  
  
  struct MapRange {
    double minimum_allowed_angle;
    double maximum_allowed_angle;
    double first_callibration_angle;
    double second_callibration_angle;
    double servo_to_first_callibration_angle;
    double servo_to_second_callibration_angle;
  };
  const MapRange _map_range;
    
  double _current_angle;
  bool _is_current_angle_set;

  double _transformArmAngleToServoAngle(double angle);

  int _nearestIntegerAngle(double angle);

  String _rangeToString();

  public:

    ServoArm(String name, Servo* servo, double length, MapRange map_range, LoggingCallback logging_callback);

    void moveTo(double angle);

    void moveBy(double delta_angle);

    bool canMoveTo(double angle);

    bool canMoveBy(double delta_angle);

    bool isAngleAllowed(double angle);

    double currentAngle();

    double minAngle() {return _map_range.minimum_allowed_angle;}

    double maxAngle() {return _map_range.maximum_allowed_angle;}

    double firstCallibrationAngle() {return _map_range.first_callibration_angle;}
    
    double secondCallibrationAngle() {return _map_range.second_callibration_angle;}

    double length();
    
};

} // namespace robotic_arm

#endif // ROBOTIC_ARM_SERVO_ARM_H
