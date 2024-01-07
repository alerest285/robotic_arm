#include "joystick.h"

namespace robotic_arm {
    
Joystick::Joystick(int horizontal_input_pin, int vertical_input_pin, double max_displacement_per_loop): 
  _horizontal_input_pin(horizontal_input_pin), _vertical_input_pin(vertical_input_pin), 
  _max_displacement_per_loop(max_displacement_per_loop){

  pinMode(_horizontal_input_pin, INPUT);  
  pinMode(_vertical_input_pin, INPUT);
}

PlaneCartesianCoordinates Joystick::getDeltaCartesianCoordinates(){
  int horizontal_input = analogRead(_horizontal_input_pin);
  int vertical_input = analogRead(_vertical_input_pin);
  int horizontal_input_in_milis = map(horizontal_input, _min_joystick_input, _max_joyistick_input, 1000, -1000);
  int vertical_input_in_milis = map(vertical_input, _min_joystick_input, _max_joyistick_input, -1000, 1000);
  double delta_x = (abs(horizontal_input_in_milis) >= _min_milis) ?
    _max_displacement_per_loop * horizontal_input_in_milis / 1000.0 : 0;
  double delta_y = (abs(vertical_input_in_milis) >= _min_milis) ?
    _max_displacement_per_loop * vertical_input_in_milis / 1000.0 : 0;
  return PlaneCartesianCoordinates({x: delta_x, y: delta_y});
}

} // namespace robotic_arm
