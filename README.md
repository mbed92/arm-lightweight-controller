# Description
The **ur3-lightweight-controller** is a very lightweight UR3 controller for a real robot that is 
based on sending URscript commands via tcp/ip. It can be used as a standalone module - 
ROS is not required.

## Requirements
This module is based purely on Python packages. Only numpy is required. Tested on Python 2.7 and 3.6. 
MoveIt is not needed to move a real robot. Whole communication is based on sending and reading 
proper bytes via sockets.

# HOWTO
See **run.py** file for example of use.

## Functionalities
Manipulator
* TODO