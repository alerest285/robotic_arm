

## Microcontroller-Driven Robotic Arm with Software Integration

![This is an alt text.](https://i.postimg.cc/R0brJh5V/Screenshot-2025-03-17-133221.png "Robot Arm")

## This Github repository is part of the C++ implementation from my Matura Thesis in Computer Science. 
> Microcontroller-Driven Robotic Arm with Software Integration 
>> Supervisor: Professor Christoph Vogel


### Abstract 
This paper exhibits the design and construction of a six-degrees-of-freedom robot arm using consumer electronics and affordable materials in order to emulate the positioning capabilities of an industrial robot arm. To achieve this, an optimal mechanical prototype is modeled and built, using servo motors as the main actuators. Furthermore, two mathematical control methods are proposed: a trigonometric exact inversion and an approximate local inversion via derivatives. These methods are implemented into an open source C++ library. The implementation successfully performed an inverse kinematic analysis, allowing for two-dimensional trajectory control and pivoting of the end-effector using input devices. A series of user interface and stability tests were conducted with positive results in precision and repeatability in most scenarios. 


## 4.6.1 Software Architecture 

> The software architecture is to be modular, making it more organized and readable. First, essential libraries for servo motor control are included (#include <Servo.h>) and utility functions for logging and enumerations for logging levels are defined. Function pointers for logging callbacks allows for monitoring the system's behavior in real-time, and is useful in the debugging process. 

## ServoArm Class 

> A class structure ServoArm is defined, with the purpose of encompassing all of the logic of controlling an “Arm” via a servo motor. Particularly ServoArm abstracts the behavior of an arm being rotated against a reference halfline. 
Key functionalities of the class include methods for moving the arm to a specified angle (moveTo), moving the arm by a delta angle (moveBy), and retrieving the current angle (currentAngle). The following is the class declaration, its implementation can be found at https://github.com/alerest285/robotic_arm/blob/main/src/servo_arm.cpp. 
 


```
class ServoArm { 
const String _name; 
const double _length; 
const int _pin; 
const LoggingCallback _logging; 
const Servo _servo; 
struct MapRange { 
double lower_bound; 
double upper_bound; 
double servo_to_lower_bound; 
double servo_to_upper_bound; 
}; 
const MapRange _map_range; 
double _current_angle; 
bool _is_current_angle_set; 
double _transformArmAngleToServoAngle(double angle); 
int _nearestIntegerAngle(double angle); 
String _rangeToString(); 
public: 
ServoArm(String name, double length, int pin, MapRange map_range, LoggingCallback logging_callback); 
void moveTo(double angle); 
void moveBy(double delta); 
double currentAngle(); 
}; 

```


## Read the paper

https://docs.google.com/document/d/1RDTosWxl55HQCBOGslz9zHXbvPj9Iu7pmoaEarzymBw/edit?usp=sharing



