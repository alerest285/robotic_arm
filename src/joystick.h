#ifndef ROBOTIC_ARM_JOYSTICK_H
#define ROBOTIC_ARM_JOYSTICK_H

#include "math.h"

namespace robotic_arm {
  
constexpr int MIN_JOYSTICK_INPUT = 0;
constexpr int MAX_JOYSTICK_INPUT = 1023;

class CartesianJoystick {

  int _horizontal_input_pin;
  int _vertical_input_pin;
  double _max_displacement_per_loop;

  const int _min_milis = 100;

  public: 

    /**
     * @brief Constructs a new MyClass instance.
     * 
     * This constructor initializes the MyClass object with provided initial values. 
     * It sets up the initial state required for the object to function correctly.
     *
     * @param horizontal_input_pin Pin number for the horizontal input signal.
     * @param vertical_input_pin Pin number for the horizontal input signal. 
     * @param max_displacement_per_loop Maximum displacement signaled (in centimeters).
     */
    CartesianJoystick(int horizontal_input_pin, int vertical_input_pin, double max_displacement_per_loop);
    
    PlaneCartesianCoordinates getDeltaCartesianCoordinates();
};