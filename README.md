# Description
The **arm-lightweight-controller** is a very lightweight robotic arm controller for a real robot that is 
based on sending commands via tcp/ip (e.g. URScript commands in case of Universal Robot arms). 
It can be used as a standalone module - ROS is not required. 

## Difference between existing drivers

### UR driver
In [ur_modern_driver](https://github.com/ros-industrial/ur_modern_driver) the MoveIt
is used to plan a path. In order to be usable in simulation, MoveIt planner 
computes many points between points specified in a trajectory. In **arm-lightweight-controller**
there is no possibility to do that - path planning is performed entirely 
in the robot's controller.

### Other robots
For now there are no other drivers implemented.

## TCP/IP communication details
Universal Robots provide the [description of TCP/IP packages](https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/remote-control-via-tcpip-16496/) that are sent from robot's controller via sockets. The current communication schema in **arm-lightweight-controller** is based on that. In order to add more functionalities from this interface - see attached file under provided link.

# Installation
In order to configure your Python environment just run the **setup.py** script.

## Usage
See **examples/run.py** file for the example of use.

## Functionality
Short description of available methods in the Manipulator class. For more details
please refer to the method descriptions in the code.
* **def move(self, trajectory, is_movej=True, is_pose=True, a=1, v=1, use_mapping=False)** - send 
a move command to the specified robot. User can specify if it is a 
linear move / points are specified in a joint space or as poses / if the trajectory is in a robot's
coordinate system or some external (see: set_mapping() method).
* **def get_pose(self)** - reads current pose from a byte stream.
* **def get_joints(self)** - reads current joint coordinates from a byte stream.
* **def set_mapping(self, matrix)** - sets a mapping between a robot coordinate 
system and some external one. While this method is invoked, user can pass trajectory in some external 
coordinate system (see: move() command).

# Contribution
All questions to: _michal.bednarek@put.poznan.pl_
