#include "logging.h"
#include "robot.h"

namespace robotic_arm {

RobotWithHorizontalHand::RobotWithHorizontalHand(LoggingCallback logging_callback): _logging(logging_callback), _method(MethodEnum::EXACT){
  _shoulder = new ServoArm("shoulder", /*length=*/ 18.7, /*pin=*/9, /*map_range=*/{0, 180, 141, 5}, _logging);
  _elbow = new ServoArm("elbow", /*length=*/ 6.7, /*pin=*/10, /*map_range=*/{90, 270, 0, 180}, _logging);      
  _hand = new ServoArm("hand", /*length=*/ 6.0, /*pin=*/11, /*map_range=*/{90, 270, 10, 170}, _logging);
  _forearm_length = 15.0;
};
  
PlaneCartesianCoordinates RobotWithHorizontalHand::_calculateCartesianCoordinates(AngularCoordinates angular_coordinates) {
  double A = _shoulder->length();
  double B = _elbow->length();
  double C = _forearm_length;
  double D = _hand->length();
  double x = A * cosDegrees(angular_coordinates.shoulder_angle) 
    - B * cosDegrees(angular_coordinates.elbow_angle + angular_coordinates.shoulder_angle)
    - C * sinDegrees(angular_coordinates.elbow_angle + angular_coordinates.shoulder_angle)
    + D;
  double y = A * sinDegrees(angular_coordinates.shoulder_angle) 
    - B * sinDegrees(angular_coordinates.elbow_angle + angular_coordinates.shoulder_angle)
    + C * cosDegrees(angular_coordinates.elbow_angle + angular_coordinates.shoulder_angle);   
  return {x: x, y: y};
}  

AngularDerivatives RobotWithHorizontalHand::_calculateAngularDerivatives(AngularCoordinates angular_coordinates) {
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

AngularCoordinates RobotWithHorizontalHand::_calculateAngularCoordinates(PlaneCartesianCoordinates cartesian_coordinates) {
  double x = cartesian_coordinates.x;
  double y = cartesian_coordinates.y;
  double A = _shoulder->length();
  double B = _elbow->length();
  double C = _forearm_length;
  double D = _hand->length();
  double R = sqrt(pow((x - D), 2) + pow(y, 2));
  double S = sqrt(pow(B, 2) + pow(C, 2));
  double shoulder_angle = acosDegrees((x - D) / R) + acosDegrees((pow(A, 2) + pow(R, 2) - pow(S, 2)) / (2 * A * R));
  double elbow_angle = acosDegrees(B / S) + acosDegrees((pow(S, 2)+ pow(A, 2)- pow(R, 2)) / (2 * A * S));
  return {shoulder_angle: shoulder_angle, elbow_angle: elbow_angle};
}

double RobotWithHorizontalHand::_calculateDeterminant(AngularDerivatives angular_derivatives){
  return angular_derivatives.x_by_shoulder_angle * angular_derivatives.y_by_elbow_angle 
     - angular_derivatives.x_by_elbow_angle * angular_derivatives.y_by_shoulder_angle;
}

void RobotWithHorizontalHand::_moveByWithExactMethod(double delta_x, double delta_y) {
  PlaneCartesianCoordinates current_cartesian_coordinates = currentCartesianCoordinates();
  PlaneCartesianCoordinates projected_cartesian_coordinates = {
    x: current_cartesian_coordinates.x + delta_x, 
    y: current_cartesian_coordinates.y + delta_y};
  AngularCoordinates projected_angular_coordinates =  
    _calculateAngularCoordinates(projected_cartesian_coordinates);
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
}


void RobotWithHorizontalHand::_moveByWithDerivativeMethod(double delta_x, double delta_y) {
  PlaneCartesianCoordinates current_cartesian_coordinates = currentCartesianCoordinates();
  PlaneCartesianCoordinates expected_cartesian_coordinates = {
    x: current_cartesian_coordinates.x + delta_x, y: current_cartesian_coordinates.y + delta_y};
  AngularDerivatives angular_derivatives = _calculateAngularDerivatives(
    {shoulder_angle: _shoulder->currentAngle(), elbow_angle: _elbow->currentAngle()});    
  double determinant = _calculateDeterminant(angular_derivatives);
  if (abs(determinant) < _differential_stability_threshold) {
      _logging(
        LoggingEnum::FATAL,
        "Currently we are in an unstable position " + current_cartesian_coordinates.toString());      
    return;
  }

  double delta_shoulder_angle = (delta_x * angular_derivatives.y_by_elbow_angle - delta_y * angular_derivatives.x_by_elbow_angle) / determinant;
  double delta_elbow_angle = (delta_y * angular_derivatives.x_by_shoulder_angle - delta_x * angular_derivatives.y_by_shoulder_angle) / determinant;
  
  AngularCoordinates projected_angular_coordinates =  {
    shoulder_angle: _shoulder->currentAngle() + delta_shoulder_angle, 
    elbow_angle: _elbow->currentAngle() + delta_elbow_angle};
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
  return;
}

double RobotWithHorizontalHand::_calculateHandAngle(AngularCoordinates angular_coordinates) {
  return angular_coordinates.shoulder_angle + angular_coordinates.elbow_angle - 90;
}

PlaneCartesianCoordinates RobotWithHorizontalHand::currentCartesianCoordinates(){
  return _calculateCartesianCoordinates(
    {shoulder_angle: _shoulder->currentAngle(), elbow_angle: _elbow->currentAngle()});
} 

AngularCoordinates RobotWithHorizontalHand::currentAngularCoordinates(){
  return {shoulder_angle: _shoulder->currentAngle(), elbow_angle: _elbow->currentAngle()};
}      

void RobotWithHorizontalHand::moveArmsTo(double shoulder_angle, double elbow_angle){
  _shoulder->moveTo(shoulder_angle);
  _elbow->moveTo(elbow_angle);
  _hand->moveTo(_calculateHandAngle({shoulder_angle, elbow_angle}));
}    

void RobotWithHorizontalHand::moveBy(double delta_x, double delta_y) {
  if (_method == MethodEnum::EXACT) {
   _moveByWithExactMethod(delta_x, delta_y);
  }
  else if (_method == MethodEnum::DERIVATIVE) {
   _moveByWithDerivativeMethod(delta_x, delta_y);
  }
}    
  
void RobotWithHorizontalHand::setMethodToExact(){
   _method = MethodEnum::EXACT;
   _logging(
    LoggingEnum::INFO,
    "Method set to EXACT"
  );
}

void RobotWithHorizontalHand::setMethodToDerivative(){
  _method = MethodEnum::DERIVATIVE;
  _logging(
    LoggingEnum::INFO,
    "Method set to DERIVATIVE"
  );
}


} // namespace robotic_arm
