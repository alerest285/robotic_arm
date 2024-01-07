#ifndef ROBOTIC_ARM_JOYSTICK_H
#define ROBOTIC_ARM_JOYSTICK_H

namespace robotic_arm {

class CartesianJoystick {

  int _horizontal_input_pin;
  int _vertical_input_pin;
  double _max_displacement_per_loop;

  const int _min_milis = 100;
  const int _min_joystick_input = 0;
  const int _max_joyistick_input = 1023;

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

} // namespace robotic_arm

#endif // ROBOTIC_ARM_LOGGING_H
