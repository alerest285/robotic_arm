#include <Arduino.h>
#include <String.h>
#include "logging.h"
#include "servo_arm.h"

namespace robotic_arm {

ServoArm::ServoArm(String name, double length, int pin, MapRange map_range, LoggingCallback logging_callback): 
  _name(name), _length(length),  _pin(pin), _map_range(map_range), _current_angle(0.0), 
  _is_current_angle_set(false), _logging(logging_callback) {
  _servo.attach(_pin);
}

double ServoArm::_transformArmAngleToServoAngle(double angle) {
  double alpha = (_map_range.upper_bound - angle) / (_map_range.upper_bound - _map_range.lower_bound);
  return alpha * _map_range.servo_to_lower_bound + (1 - alpha) * _map_range.servo_to_upper_bound;
}  

int ServoArm::_nearestIntegerAngle(double angle) {
  return floor(angle + 0.5);
}

String ServoArm::_rangeToString() {
  return  "[" + String(_map_range.lower_bound) + ", " + String(_map_range.upper_bound) + "]";
}

void ServoArm::moveTo(double angle){
  if ((angle < _map_range.lower_bound) || (angle > _map_range.upper_bound)) {
    // String str_range = "[" + String(_map_range.lower_bound) + ", " + String(_map_range.upper_bound) + "]";
    _logging(
      LoggingEnum::WARN, 
      "Servo arm " + String(_name) + ". Trying to move to an angle out of range " + _rangeToString() + ": (" + String(angle) + ").");                      
    return;        
  }

  _current_angle = angle;
  if (!_is_current_angle_set) {
    _logging(
      LoggingEnum::INFO, 
      "Servo arm " + String(_name) + ". Initial angle set to " + String(_current_angle) + ".");              
  }
  _is_current_angle_set = true;
  int servo_angle = ServoArm::_nearestIntegerAngle(ServoArm::_transformArmAngleToServoAngle(_current_angle));
  _logging(
    LoggingEnum::INFO,
    "Moving arm " + _name + " to position " + String(_current_angle) + " degrees via servo write " + String(servo_angle) + " degrees."
  );
  _servo.write(servo_angle);
}

void ServoArm::moveBy(double delta) {
  if (!_is_current_angle_set) {
    _logging(
      LoggingEnum::ERROR, 
      "Servo arm " + String(_name) + ". Trying to call moveBy without setting the arm to an initial angle.");
    return;
  }
  moveTo(_current_angle + delta);
}

double ServoArm::currentAngle() {
  if (!_is_current_angle_set) {
    _logging(
      LoggingEnum::ERROR, 
      "Servo arm " + String(_name) + ". Trying to call currentAngle without setting the arm to an initial angle.");
    return -1;
  }
  return _current_angle;
}

double ServoArm::length() {
  return _length;
}

} // namespace robotic_arm
