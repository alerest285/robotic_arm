#include <Arduino.h>
#include <String.h>
#include "logging.h"
#include "servo_arm.h"

namespace robotic_arm {

ServoArm::ServoArm(String name, Servo* servo, double length, MapRange map_range, LoggingCallback logging_callback): 
  _name(name), _servo(servo), _length(length), _map_range(map_range), _current_angle(0.0), 
  _is_current_angle_set(false), _logging(logging_callback) {
}

double ServoArm::_transformArmAngleToServoAngle(double angle) {
  double alpha = (_map_range.second_callibration_angle - angle) / (_map_range.second_callibration_angle - _map_range.first_callibration_angle);
  return alpha * _map_range.servo_to_first_callibration_angle + (1 - alpha) * _map_range.servo_to_second_callibration_angle;
}  

int ServoArm::_nearestIntegerAngle(double angle) {
  return floor(angle + 0.5);
}

String ServoArm::_rangeToString() {
  return  "[" + String(_map_range.minimum_allowed_angle) + ", " + String(_map_range.maximum_allowed_angle) + "]";
}

bool ServoArm::isAngleAllowed(double angle){
  return (angle >= _map_range.minimum_allowed_angle) && (angle <= _map_range.maximum_allowed_angle);
}

bool ServoArm::canMoveTo(double angle){
  if (isAngleAllowed(angle)) {
    return true;
  }
  _logging(
    LoggingEnum::INFO, 
    "Servo arm " + String(_name) + ". Can't move to angle " + String(angle)  + ". It is out of range " + _rangeToString() + ".");      
  return false;
}

bool ServoArm::canMoveBy(double delta_angle){
  if (!_is_current_angle_set) {
    _logging(
      LoggingEnum::INFO, 
      "Servo arm " + String(_name) + ". Can't apply moveBy without setting an initial angle.");
    return false;
  }
  double resulting_angle = _current_angle + delta_angle;
  if (isAngleAllowed(resulting_angle)) {
    return true;
  }
  _logging(
    LoggingEnum::INFO, 
    "Servo arm " + String(_name) + ". Can't move by " + String(resulting_angle) + " to angle " + String(resulting_angle)  + ". It is out of range " + _rangeToString() + ".");      
  return false;
}

void ServoArm::moveTo(double angle){
  if (!canMoveTo(angle)) {
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
  _servo->write(servo_angle);
}

void ServoArm::moveBy(double delta_angle) {
  if (!canMoveBy(delta_angle)) {
    return;
  }
  moveTo(_current_angle + delta_angle);
}

double ServoArm::currentAngle() {
  if (!_is_current_angle_set) {
    _logging(
      LoggingEnum::ERROR, 
      "Servo arm " + String(_name) + ". Trying to call currentAngle without setting the arm to an initial angle.");
    return 0;
  }
  return _current_angle;
}

double ServoArm::length() {
  return _length;
}

} // namespace robotic_arm
