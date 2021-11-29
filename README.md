[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Build Status](https://app.travis-ci.com/aswathselvam/swarm_robots.svg?branch=main)
[![Coverage Status](https://coveralls.io/repos/github/aswathselvam/swarm_robots/badge.svg?branch=main)](https://coveralls.io/github/aswathselvam/swarm_robots?branch=main)

# Jackal Rescue Robot

This project is an implementation of a rescue operation using a swarm of [Clearpath Jackal robots](https://clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/).
It is built using ROS 1 and C++

## 

## Overview
Search and rescue is one of the most widely implemented problems in real-life robotic applications. The task of rescue can be generalized and used for moving an object from a source to any desired destination. In this project, we propose to develop a system that will deploy a swarm of 20 robots in a simulated gazebo environment and help rescue an object successfully to the defined end goal. The robot we propose to use is the Jackal unmanned ground vehicle [1]. Agents in the rescue swarm will spawn at a pre-set start position and then navigate towards the object to be rescued. Once they reach the object contact point the agents will push the object synchronously and start moving towards the goal location.

<span align="centre">
<img src="https://clearpathrobotics.com/wp-content/uploads/2015/06/front.jpg" alt="cool Robot"></img>

[source](https://clearpathrobotics.com/wp-content/uploads/2015/06/front.jpg)</span>

## Authors
<div style="display:grid;grid-template-columns:auto auto">
<div>
   <p> <b>1. Kavyashree Devadiga (117398045)</b> </br>Computer buff, interested in smart autonomous systems. Has a Bachelors degree in Computer Science and is currently pursuing Masters of Engineering in Robotics at University of Maryland College Park.
      <a href="https://www.linkedin.com/in/kavyashree-devadiga/" title="LinkedIn" rel="nofollow noreferrer">
      <img src="https://i.stack.imgur.com/gVE0j.png" alt="linkedin">
      </a> &nbsp; 
      <a href="https://github.com/kavyadevd" rel="nofollow noreferrer" title="Github">
      <img src="https://i.imgur.com/J6LeoUb.png" width="19px" alt="github">
      </a>
   </p>
</div>
<div>
   <p> <b>2. Aswath Muthuselvam (118286204)</b></br>Holds interest in Autonomous Mobile Agents, Computer Vision, Simultaneous Localization and Mapping, AI, Real-Time systems, and controls. Has a Bachelors's degree in Electrical, Electronics, and Communications Engineering and is currently pursuing Masters of Engineering in Robotics at the University of Maryland College Park.
      <a href="https://www.linkedin.com/in/aswath-m/" title="LinkedIn" rel="nofollow noreferrer">
      <img src="https://i.stack.imgur.com/gVE0j.png" alt="linkedin">
      </a> &nbsp; 
      <a href="https://github.com/aswathselvam" title="Github" rel="nofollow noreferrer">
      <img src="https://i.imgur.com/J6LeoUb.png" width="19px" alt="github">
      </a>
   </p>
   </div
</div>

## UML

### Activity diagram
<img src="https://github.com/kavyadevd/swarm_robots/blob/test/UML/ActivityDiagram.png?raw=true" alt="Activity Diagram" width="500px"></img>

### Class diagram
<img src="https://github.com/kavyadevd/swarm_robots/blob/test/UML/Class%20diagram.png?raw=true" width="500px" alt="Class Diagram"></img>


### Assumptions:
1. Robots are not aware of its neighbours.
2. Centralized communication, there is a common node(rosmaster) through which all robots communicates with each other. 
3. Swarms only operate in 2D plane.

### Tested on:
1. Ubuntu 20.04 and Ubuntu 18.04
2. OMPL library version: 1.5.2 

### Development process
The project will be developed using industry-grade agile methodologies. The agile method being adaptive in nature will quickly adapt to software requirements and changes due to challenges faced if any during the project development cycle. For software development, pair programming strategies will be used which will ensure a robust, bug-free package. We have hierarchical test suites, which will test the submodules by implementing unit test cases and the validity of the whole system. To simulate real-world challenges we plan to use different Gmock test cases and confirm the correctness of the system.

[Meeting notes](https://docs.google.com/document/d/1nNpMe6DLzv8XDJHyTaXOK77MExHqAslBlWCvYTAc9Zk/edit?usp=sharing)

[AIP Spreadsheet](https://docs.google.com/spreadsheets/d/1eQ78AiMMgUXJpQEjbjUjjJoJ0I1oSbPfRSU09nT6VKE/edit?usp=sharing)

## Licensing
The project is licensed under the [3-Clause BSD License](https://opensource.org/licenses/BSD-3-Clause). Click [here](https://github.com/kavyadevd/swarm_robots/blob/main/LICENSE) to know more
