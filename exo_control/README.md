# Exo_control 

I will consider design decisions here. Currently this package has a virtual joystick that will create velocity commands that are given to the gait generator controller. The gait generator and the balance controller will send joint velocity commands to a topic. 

Essentially we need to create a stack-of-tasks controller that will do the following tasks  
- Keep torso upright (highest priority).
- Regulate the capture point for balance (medium).
- Track the nominal gait joint trajectory (lowest).

## Issues

The code needs to run on a real time system which windows and mac don't support. We need to find a linux machine or use the [raspberry pi](https://github.com/ros-realtime/ros-realtime-rpi4-image). 

## Gazebo

The gazebo 