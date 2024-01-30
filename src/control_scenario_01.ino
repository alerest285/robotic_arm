#include <Arduino.h>
#include <HardwareSerial.h>
#include "joystick.h"
#include "logging.h"
#include "robot.h"

#define HORZ_PIN A0
#define VERT_PIN A1
#define ANGLE_PIN A2

void logging(robotic_arm::LoggingEnum level, String message) {
  Serial.println(LoggingEnumToString(level) + ": " + message);
}

// For some reason servo attachment can't be handled inside the class,
// so these must be initialized outside the class.
Servo shoulder_servo;
Servo elbow_servo;
Servo hand_servo;
robotic_arm::ServoArm shoulder("shoulder", &shoulder_servo, /*length=*/ 18.7, /*map_range=*/{10, 180, 90, 180, 95, 5}, logging);
robotic_arm::ServoArm elbow("elbow", &elbow_servo, /*length=*/ 6.7, /*map_range=*/{110, 270, 180, 270, 85, 180}, logging);      
robotic_arm::ServoArm hand("hand", &hand_servo, /*length=*/ 6.0, /*map_range=*/{115, 265, 180, 265, 78, 175}, logging);
robotic_arm::Robot robot(&shoulder, &elbow, &hand, logging);

robotic_arm::CartesianJoystick cartesian_joystick(HORZ_PIN, VERT_PIN, /*max_displacement_per_loop=*/0.1);
robotic_arm::AngularJoystick angular_joystick(ANGLE_PIN, /*max_displacement_per_loop=*/2);

int loop_counter = 0;
const int SECONDS_PER_LOOP = 1000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);   
  shoulder_servo.attach(9);
  elbow_servo.attach(10);
  hand_servo.attach(11);  
}

void loop() {
  // put your main code here, to run repeatedly:
logging(
    robotic_arm::LoggingEnum::INFO,
    "\nLoop No " + String(loop_counter) + "."
  );
  switch (loop_counter) {
    case 0: 
      // INITIALIZATION
      // robot.setMethodToDerivative();
      robot.setMethodToExact();
      robot.moveArmsTo({shoulder_angle: 80, elbow_angle: 180, hand_reference_angle: 0});
      logging(
        robotic_arm::LoggingEnum::INFO, 
        "Cartesian coordinates: " + robot.currentCartesianCoordinates().toString()
      );
      logging(
        robotic_arm::LoggingEnum::INFO, 
        "Angular coordinates: " + robot.currentAngularCoordinates().toString()
      );
      break;
    default: 
      // DEFAULT EXECUTION LOOP

      // Retrieve the delta coordinates from the cartesian joystick
      robotic_arm::PlaneCartesianCoordinates delta_coordinates = cartesian_joystick.getDeltaCartesianCoordinates();
      logging(
        robotic_arm::LoggingEnum::DEBUG, 
        "delta x: " + String(delta_coordinates.x) + ", delta y: " + String(delta_coordinates.y));

      // Retrieve the hand angle from the angular joystick
      double delta_hand_reference_angle = angular_joystick.getDeltaAngle();
      logging(
        robotic_arm::LoggingEnum::DEBUG, 
        "delta angle: " + String(delta_hand_reference_angle));      
      robot.moveBy(delta_coordinates);
      logging(
        robotic_arm::LoggingEnum::INFO, 
        "Cartesian coordinates: " + robot.currentCartesianCoordinates().toString()
      );
      logging(
        robotic_arm::LoggingEnum::INFO, 
        "Angular coordinates: " + robot.currentAngularCoordinates().toString()
      );
      robot.rotateHandBy(delta_hand_reference_angle);
      logging(
        robotic_arm::LoggingEnum::INFO, 
        "Cartesian coordinates: " + robot.currentCartesianCoordinates().toString()
      );
      logging(
        robotic_arm::LoggingEnum::INFO, 
        "Angular coordinates: " + robot.currentAngularCoordinates().toString()
      );      
      break;
  }

  // 
  delay(SECONDS_PER_LOOP); 
  loop_counter++;
}
