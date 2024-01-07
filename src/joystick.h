#ifndef ROBOTIC_ARM_JOYSTICK_H
#define ROBOTIC_ARM_JOYSTICK_H

namespace robotic_arm {

class Joystick {

  int _horizontal_input_pin;
  int _vertical_input_pin;
  double _max_displacement_per_loop;

  const int _min_milis = 100;
  const int _min_joystick_input = 0;
  const int _max_joyistick_input = 1023;

  public: 
    
    Joystick(int horizontal_input_pin, int vertical_input_pin, double max_displacement_per_loop);

    PlaneCartesianCoordinates getDeltaCartesianCoordinates();
};

} // namespace robotic_arm

#endif // ROBOTIC_ARM_LOGGING_H
