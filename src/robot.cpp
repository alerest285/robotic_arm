#include <Arduino.h>
#include <String.h>
#include "logging.h"
#include "math.h"
#include "robot.h"

namespace robotic_arm {

Robot::Robot(ServoArm* shoulder_arm, ServoArm* elbow_arm, ServoArm* hand_arm, LoggingCallback logging_callback): 
  _shoulder(shoulder_arm), _elbow(elbow_arm), _hand(hand_arm),
  _logging(logging_callback), _method(MethodEnum::EXACT), _forearm_length(15.0){};

String Robot::AngularCoordinates::toString(){
  return "shoulder_angle: " + String(shoulder_angle) 
    + ", elbow_angle: " + String(elbow_angle)
    + ", hand_reference_angle: " + String(hand_reference_angle);
}  
  
PlaneCartesianCoordinates Robot::_calculateCartesianCoordinates(AngularCoordinates angular_coordinates) {
  double A = _shoulder->length();
  double B = _elbow->length();
  double C = _forearm_length;
  double D = _hand->length();
  double x = A * cosDegrees(angular_coordinates.shoulder_angle) 
    - B * cosDegrees(angular_coordinates.elbow_angle + angular_coordinates.shoulder_angle)
    - C * sinDegrees(angular_coordinates.elbow_angle + angular_coordinates.shoulder_angle)
    + D * cosDegrees(angular_coordinates.hand_reference_angle);
  double y = A * sinDegrees(angular_coordinates.shoulder_angle) 
    - B * sinDegrees(angular_coordinates.elbow_angle + angular_coordinates.shoulder_angle)
    + C * cosDegrees(angular_coordinates.elbow_angle + angular_coordinates.shoulder_angle)
    + D * sinDegrees(angular_coordinates.hand_reference_angle);   
  return {x: x, y: y};
}  

Robot::AngularDerivatives Robot::_calculateAngularDerivatives(
  Robot::AngularCoordinates angular_coordinates) {
  double A = _shoulder->length();
  double B = _elbow->length();
  double C = _forearm_length;
  double D = _hand->length();
  double x_by_shoulder_angle = A * cosDegreesDerivative(angular_coordinates.shoulder_angle) 
    - B * cosDegreesDerivative(angular_coordinates.elbow_angle + angular_coordinates.shoulder_angle)
    - C * sinDegreesDerivative(angular_coordinates.elbow_angle + angular_coordinates.shoulder_angle);
  double x_by_elbow_angle = - B * cosDegreesDerivative(angular_coordinates.elbow_angle + angular_coordinates.shoulder_angle)
    - C * sinDegreesDerivative(angular_coordinates.elbow_angle + angular_coordinates.shoulder_angle);
  double y_by_shoulder_angle = A * sinDegreesDerivative(angular_coordinates.shoulder_angle) 
    - B * sinDegreesDerivative(angular_coordinates.elbow_angle + angular_coordinates.shoulder_angle)
    + C * cosDegreesDerivative(angular_coordinates.elbow_angle + angular_coordinates.shoulder_angle); 
  double y_by_elbow_angle = - B * sinDegreesDerivative(angular_coordinates.elbow_angle + angular_coordinates.shoulder_angle)
    + C * cosDegreesDerivative(angular_coordinates.elbow_angle + angular_coordinates.shoulder_angle);
  return {
    x_by_shoulder_angle: x_by_shoulder_angle,
    x_by_elbow_angle: x_by_elbow_angle,
    y_by_shoulder_angle: y_by_shoulder_angle,
    y_by_elbow_angle: y_by_elbow_angle
  };
}

Robot::AngularCoordinates Robot::_calculateAngularCoordinates(
  PlaneCartesianCoordinates cartesian_coordinates, double hand_reference_angle) {
  double A = _shoulder->length();
  double B = _elbow->length();
  double C = _forearm_length;
  double D = _hand->length();

  // Coordinates of the end point of the forearm.
  double x_prime = cartesian_coordinates.x - D * cosDegrees(hand_reference_angle); 
  double y_prime = cartesian_coordinates.y - D * sinDegrees(hand_reference_angle);
  
  double R = sqrt(pow(x_prime, 2) + pow(y_prime, 2));
  double S = sqrt(pow(B, 2) + pow(C, 2));

  double shoulder_angle = acosDegrees(x_prime / R) + acosDegrees((pow(A, 2) + pow(R, 2) - pow(S, 2)) / (2 * A * R));
  double elbow_angle = acosDegrees(B / S) + acosDegrees((pow(S, 2)+ pow(A, 2)- pow(R, 2)) / (2 * A * S));
  return {shoulder_angle: shoulder_angle, elbow_angle: elbow_angle, hand_reference_angle: hand_reference_angle};
}

double Robot::_calculateDeterminant(AngularDerivatives angular_derivatives){
  return angular_derivatives.x_by_shoulder_angle * angular_derivatives.y_by_elbow_angle 
     - angular_derivatives.x_by_elbow_angle * angular_derivatives.y_by_shoulder_angle;
}

void Robot::_moveByWithExactMethod(
  PlaneCartesianCoordinates delta_cartesian_coordinates,
  double delta_hand_reference_angle) {
  // Current state.
  PlaneCartesianCoordinates current_cartesian_coordinates = currentCartesianCoordinates();
  double hand_reference_angle = _getCurrentHandReferenceAngle();

  // Projected state.
  PlaneCartesianCoordinates projected_cartesian_coordinates = {
    x: current_cartesian_coordinates.x + delta_cartesian_coordinates.x, 
    y: current_cartesian_coordinates.y + delta_cartesian_coordinates.y};
  double projected_hand_reference_angle = hand_reference_angle + delta_hand_reference_angle;
  AngularCoordinates projected_angular_coordinates =  
    _calculateAngularCoordinates(projected_cartesian_coordinates, projected_hand_reference_angle);
  if (isnan(projected_angular_coordinates.shoulder_angle) 
    || isnan(projected_angular_coordinates.elbow_angle)) {
    _logging(
    LoggingEnum::WARN,
      "Trying to move the robot to the impossible position " 
        + projected_cartesian_coordinates.toString() 
        + "with a hand reference angle " + String(projected_hand_reference_angle));
    return;
  }
  moveArmsTo(projected_angular_coordinates);
}

void Robot::_moveByWithDerivativeMethod(
  PlaneCartesianCoordinates delta_cartesian_coordinates,
  double delta_hand_reference_angle){
  double D = _hand->length();

  // Current state.
  PlaneCartesianCoordinates current_cartesian_coordinates = currentCartesianCoordinates();
  double hand_reference_angle = _getCurrentHandReferenceAngle();

  // Expected state.
  PlaneCartesianCoordinates expected_cartesian_coordinates = {
    x: current_cartesian_coordinates.x + delta_cartesian_coordinates.x, 
    y: current_cartesian_coordinates.y + delta_cartesian_coordinates.y};

  // Jacobian calculation.
  AngularDerivatives angular_derivatives = _calculateAngularDerivatives(
    {shoulder_angle: _shoulder->currentAngle(), elbow_angle: _elbow->currentAngle()});    
  double determinant = _calculateDeterminant(angular_derivatives);
  if (abs(determinant) < _differential_stability_threshold) {
      _logging(
        LoggingEnum::FATAL,
        "Currently we are in an unstable position " + current_cartesian_coordinates.toString());      
    return;
  }

  // Required deltas.
  double forearm_delta_x = delta_cartesian_coordinates.x + D * delta_hand_reference_angle * cosDegreesDerivative(hand_reference_angle);
  double forearm_delta_y = delta_cartesian_coordinates.y + D * delta_hand_reference_angle * sinDegreesDerivative(hand_reference_angle);
  double delta_shoulder_angle = (
    forearm_delta_x * angular_derivatives.y_by_elbow_angle 
    - forearm_delta_y * angular_derivatives.x_by_elbow_angle) / determinant;
  double delta_elbow_angle = (
    forearm_delta_y * angular_derivatives.x_by_shoulder_angle 
    - forearm_delta_x * angular_derivatives.y_by_shoulder_angle) / determinant;
  
  // Projected state.
  Robot::AngularCoordinates projected_angular_coordinates =  {
    shoulder_angle: _shoulder->currentAngle() + delta_shoulder_angle, 
    elbow_angle: _elbow->currentAngle() + delta_elbow_angle,
    hand_reference_angle: hand_reference_angle + delta_hand_reference_angle};
  PlaneCartesianCoordinates projected_cartesian_coordinates = _calculateCartesianCoordinates(projected_angular_coordinates);
  _logging(
    LoggingEnum::INFO,
    "Target: " + expected_cartesian_coordinates.toString() + ". Actual: " + projected_cartesian_coordinates.toString());

  // Jacobian of the projected state.
  AngularDerivatives projected_angular_derivatives = _calculateAngularDerivatives(projected_angular_coordinates);    
  double projected_determinant = _calculateDeterminant(projected_angular_derivatives);
  if (abs(projected_determinant) < _differential_stability_threshold) {
      _logging(
        LoggingEnum::WARN,
        "Trying to move to an unstable position " + projected_cartesian_coordinates.toString());      
    return;
  }
  double delta_hand_angle = delta_shoulder_angle + delta_elbow_angle - delta_hand_reference_angle;
  if (_shoulder->canMoveBy(delta_shoulder_angle) 
    && _elbow->canMoveBy(delta_elbow_angle) 
    && _hand->canMoveBy(delta_hand_angle)) {
    _shoulder->moveBy(delta_shoulder_angle);
    _elbow->moveBy(delta_elbow_angle);
    _hand->moveBy(delta_hand_angle);
  }
}

double Robot::_calculateHandAngle(Robot::AngularCoordinates angular_coordinates) {
  return angular_coordinates.shoulder_angle + angular_coordinates.elbow_angle 
    - angular_coordinates.hand_reference_angle - 90;
}

double Robot::_getCurrentHandReferenceAngle(){
  return _shoulder->currentAngle() + _elbow->currentAngle() - _hand->currentAngle() - 90;
}

PlaneCartesianCoordinates Robot::currentCartesianCoordinates(){
  return _calculateCartesianCoordinates(currentAngularCoordinates());
} 

Robot::AngularCoordinates Robot::currentAngularCoordinates(){
  return {
    shoulder_angle: _shoulder->currentAngle(), 
    elbow_angle: _elbow->currentAngle(),
    hand_reference_angle: _getCurrentHandReferenceAngle()};
}      

void Robot::moveArmsTo(AngularCoordinates angular_coordinates){
  double hand_angle = _calculateHandAngle(angular_coordinates);
  if(_shoulder->canMoveTo(angular_coordinates.shoulder_angle) 
    && _elbow->canMoveTo(angular_coordinates.elbow_angle) 
    && _hand->canMoveTo(hand_angle)){
    _shoulder->moveTo(angular_coordinates.shoulder_angle);
    _elbow->moveTo(angular_coordinates.elbow_angle);
    _hand->moveTo(hand_angle);
  }  
}    

void Robot::moveBy(PlaneCartesianCoordinates delta_cartesian_coordinates) {
  if (_method == MethodEnum::EXACT) {
   _moveByWithExactMethod(delta_cartesian_coordinates, /*delta_hand_reference_angle=*/0);
  }
  else if (_method == MethodEnum::DERIVATIVE) {
   _moveByWithDerivativeMethod(delta_cartesian_coordinates, /*delta_hand_reference_angle=*/0);
  }
}   

void Robot::rotateHandBy(double delta_hand_reference_angle) {
  if (_method == MethodEnum::EXACT) {
   _moveByWithExactMethod(/*delta_cartesian_coordinates=*/{x: 0, y: 0}, delta_hand_reference_angle);
  }
  else if (_method == MethodEnum::DERIVATIVE) {
   _moveByWithExactMethod(/*delta_cartesian_coordinates=*/{x: 0, y: 0}, delta_hand_reference_angle);
  }
}
  
void Robot::setMethodToExact(){
   _method = MethodEnum::EXACT;
   _logging(
    LoggingEnum::INFO,
    "Method set to EXACT"
  );
}

void Robot::setMethodToDerivative(){
  _method = MethodEnum::DERIVATIVE;
  _logging(
    LoggingEnum::INFO,
    "Method set to DERIVATIVE"
  );
}

} // namespace robotic_arm
