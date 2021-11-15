# Husky Rescue Robot

This project is an implementation of a rescue operation using a swarm of [Clearpath Husky robots](https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/).
It is built using ROS 1 and C++

## 

<span align="centre">
<img src="https://clearpathrobotics.com/wp-content/uploads/2015/07/clearpath_1.jpg" alt="cool Robot"></img>

[source](https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/)</span>


### Assumptions:
1. Robots are not aware of its neighbours.
2. Centralized communication, there is a common node(rosmaster) through which all robots communicates with each other. 
3. Swarms only operate in 2D plane.

### Tested on:
1. Ubuntu 20.04 and Ubuntu 18.04
2. OMPL library version: 
