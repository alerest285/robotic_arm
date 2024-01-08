#include <Arduino.h>
#include <String.h>
#include "logging.h"
#include "math.h"
#include "robot.h"

namespace robotic_arm {

Robot::Robot(LoggingCallback logging_callback): _logging(logging_callback), _method(MethodEnum::EXACT), 
  _forearm_length(15.0), _hand_reference_angle(0.0){
  _shoulder = new ServoArm("shoulder", /*length=*/ 18.7, /*pin=*/9, /*map_range=*/{0, 180, 141, 5}, _logging);
  _elbow = new ServoArm("elbow", /*length=*/ 6.7, /*pin=*/10, /*map_range=*/{90, 270, 0, 180}, _logging);      
  _hand = new ServoArm("hand", /*length=*/ 6.0, /*pin=*/11, /*map_range=*/{90, 270, 10, 170}, _logging);
};

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

void Robot::_moveByWithExactMethod(PlaneCartesianCoordinates delta_cartesian_coordinates) {
  PlaneCartesianCoordinates current_cartesian_coordinates = currentCartesianCoordinates();
  PlaneCartesianCoordinates projected_cartesian_coordinates = {
    x: current_cartesian_coordinates.x + delta_cartesian_coordinates.x, 
    y: current_cartesian_coordinates.y + delta_cartesian_coordinates.y};
  AngularCoordinates projected_angular_coordinates =  
    _calculateAngularCoordinates(projected_cartesian_coordinates, _hand_reference_angle);
  if (isnan(projected_angular_coordinates.shoulder_angle) 
    || isnan(projected_angular_coordinates.elbow_angle)) {
    _logging(
    LoggingEnum::WARN,
      "Moving the robot to the impossible position " 
        + projected_cartesian_coordinates.toString());
    return;
  }
  _shoulder->moveTo(projected_angular_coordinates.shoulder_angle);
  _elbow->moveTo(projected_angular_coordinates.elbow_angle);
  _hand->moveTo(_calculateHandAngle(projected_angular_coordinates));
  // Necessary in case then hand arm reaches its limit:
  _updateHandReferenceAngle();
}

void Robot::_moveByWithDerivativeMethod(PlaneCartesianCoordinates delta_cartesian_coordinates) {
  PlaneCartesianCoordinates current_cartesian_coordinates = currentCartesianCoordinates();
  PlaneCartesianCoordinates expected_cartesian_coordinates = {
    x: current_cartesian_coordinates.x + delta_cartesian_coordinates.x, 
    y: current_cartesian_coordinates.y + delta_cartesian_coordinates.y};
  AngularDerivatives angular_derivatives = _calculateAngularDerivatives(
    {shoulder_angle: _shoulder->currentAngle(), elbow_angle: _elbow->currentAngle()});    
  double determinant = _calculateDeterminant(angular_derivatives);
  if (abs(determinant) < _differential_stability_threshold) {
      _logging(
        LoggingEnum::FATAL,
        "Currently we are in an unstable position " + current_cartesian_coordinates.toString());      
    return;
  }

  double delta_shoulder_angle = (
    delta_cartesian_coordinates.x * angular_derivatives.y_by_elbow_angle 
    - delta_cartesian_coordinates.y * angular_derivatives.x_by_elbow_angle) / determinant;
  double delta_elbow_angle = (
    delta_cartesian_coordinates.y * angular_derivatives.x_by_shoulder_angle 
    - delta_cartesian_coordinates.x * angular_derivatives.y_by_shoulder_angle) / determinant;
  
  Robot::AngularCoordinates projected_angular_coordinates =  {
    shoulder_angle: _shoulder->currentAngle() + delta_shoulder_angle, 
    elbow_angle: _elbow->currentAngle() + delta_elbow_angle,
    hand_reference_angle: _hand_reference_angle};
  PlaneCartesianCoordinates projected_cartesian_coordinates = _calculateCartesianCoordinates(projected_angular_coordinates);
  _logging(
    LoggingEnum::INFO,
    "Target: " + expected_cartesian_coordinates.toString() + ". Actual: " + projected_cartesian_coordinates.toString());
  AngularDerivatives projected_angular_derivatives = _calculateAngularDerivatives(projected_angular_coordinates);    
  double projected_determinant = _calculateDeterminant(projected_angular_derivatives);
  if (abs(projected_determinant) < _differential_stability_threshold) {
      _logging(
        LoggingEnum::WARN,
        "Trying to move to an unstable position " + projected_cartesian_coordinates.toString());      
    return;
  }
  _shoulder->moveBy(delta_shoulder_angle);
  _elbow->moveBy(delta_elbow_angle);
  _hand->moveBy(delta_shoulder_angle + delta_elbow_angle);
  _updateHandReferenceAngle();
  return;
}

double Robot::_calculateHandAngle(Robot::AngularCoordinates angular_coordinates) {
  return angular_coordinates.shoulder_angle + angular_coordinates.elbow_angle 
    - angular_coordinates.hand_reference_angle - 90;
}

void Robot::_updateHandReferenceAngle(){
  _hand_reference_angle = _shoulder->currentAngle() + _elbow->currentAngle() - _hand->currentAngle() - 90;
}

PlaneCartesianCoordinates Robot::currentCartesianCoordinates(){
  return _calculateCartesianCoordinates(
    {shoulder_angle: _shoulder->currentAngle(), elbow_angle: _elbow->currentAngle()});
} 

Robot::AngularCoordinates Robot::currentAngularCoordinates(){
  return {
    shoulder_angle: _shoulder->currentAngle(), 
    elbow_angle: _elbow->currentAngle(),
    hand_reference_angle: _hand_reference_angle};
}      

void Robot::moveArmsTo(AngularCoordinates angular_coordinates){
  _shoulder->moveTo(angular_coordinates.shoulder_angle);
  _elbow->moveTo(angular_coordinates.elbow_angle);
  _hand->moveTo(_calculateHandAngle(angular_coordinates));
  _updateHandReferenceAngle();
}    

void Robot::moveBy(PlaneCartesianCoordinates delta_cartesian_coordinates) {
  if (_method == MethodEnum::EXACT) {
   _moveByWithExactMethod(delta_cartesian_coordinates);
  }
  else if (_method == MethodEnum::DERIVATIVE) {
   _moveByWithDerivativeMethod(delta_cartesian_coordinates);
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
