#include <Arduino.h>
#include <HardwareSerial.h>
#include "joystick.h"
#include "logging.h"
#include "robot.h"

#define HORZ_PIN A0
#define VERT_PIN A1

void logging(robotic_arm::LoggingEnum level, String message) {
  Serial.println(robotic_arm::LoggingEnumToString(level) + ": " + message);
}

robotic_arm::Robot robot(logging);
robotic_arm::CartesianJoystick joystick(HORZ_PIN, VERT_PIN, /*max_displacement_per_loop=*/0.1);

int loop_counter = 0;
const int SECONDS_PER_LOOP = 5000;

void setup() {
  Serial.begin(9600);    
}

void loop() {

  logging(
    robotic_arm::LoggingEnum::INFO,
    "\nLoop No " + String(loop_counter) + "."
  );
  switch (loop_counter) {
    case 0: 
      // INITIALIZATION
      robot.setMethodToDerivative();
      robot.moveArmsTo(90, 180, 0);
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
      robotic_arm::PlaneCartesianCoordinates delta_coordinates = joystick.getDeltaCartesianCoordinates();
      logging(
        robotic_arm::LoggingEnum::DEBUG, 
        "delta x: " + String(delta_coordinates.x) + ", delta y: " + String(delta_coordinates.y));
      robot.moveBy(delta_coordinates);
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
