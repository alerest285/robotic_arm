#ifndef ROBOTIC_ARM_ROBOT_H
#define ROBOTIC_ARM_ROBOT_H

#include <String.h>
#include "logging.h"
#include "math.h"
#include "servo_arm.h"

namespace robotic_arm {

class Robot {

  LoggingCallback _logging;

  ServoArm *_shoulder;
  ServoArm *_elbow;
  ServoArm *_hand;

  const double _forearm_length;

  const double _differential_stability_threshold = 1e-3;

  struct AngularCoordinates {
    double shoulder_angle;
    double elbow_angle;
    double hand_reference_angle;
    String toString();
  };

  struct AngularDerivatives {
    double x_by_shoulder_angle;
    double x_by_elbow_angle;
    double y_by_shoulder_angle;
    double y_by_elbow_angle;    
  };

  enum class MethodEnum {
    EXACT, 
    DERIVATIVE
  };

  MethodEnum _method;
  
  PlaneCartesianCoordinates _calculateCartesianCoordinates(AngularCoordinates angular_coordinates);

  AngularDerivatives _calculateAngularDerivatives(AngularCoordinates angular_coordinates);

  AngularCoordinates _calculateAngularCoordinates(PlaneCartesianCoordinates cartesian_coordinates, double hand_reference_angle);

  double _calculateDeterminant(AngularDerivatives angular_derivatives);

  void _moveByWithExactMethod(PlaneCartesianCoordinates delta_cartesian_coordinates);

  void _moveByWithDerivativeMethod(PlaneCartesianCoordinates delta_cartesian_coordinates);

  double _calculateHandAngle(AngularCoordinates angular_coordinates);

  double _getCurrentHandReferenceAngle();

  public:

    Robot(LoggingCallback logging_callback);
    
    PlaneCartesianCoordinates currentCartesianCoordinates();
    
    AngularCoordinates currentAngularCoordinates();
    
    void moveArmsTo(AngularCoordinates angular_coordinates);
    
    void moveBy(PlaneCartesianCoordinates delta_cartesian_coordinates);
    
    void setMethodToExact();
    
    void setMethodToDerivative();

};

} // namespace robotic_arm

#endif // ROBOTIC_ARM_ROBOT_H

