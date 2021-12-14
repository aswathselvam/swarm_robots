
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Build Status](https://app.travis-ci.com/aswathselvam/swarm_robots.svg?branch=main)](https://app.travis-ci.com/aswathselvam/swarm_robots)
[![Coverage Status](https://coveralls.io/repos/github/aswathselvam/swarm_robots/badge.svg?branch=main)](https://coveralls.io/github/aswathselvam/swarm_robots?branch=main)

# Jackal Rescue Robot

This project is an implementation of a rescue operation using a swarm of [Clearpath Jackal robots](https://clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/).


## Overview
Search and rescue is one of the most widely implemented problems in real-life robotic applications. The task of rescue can be generalized and used for moving an object from a source to any desired destination. In this project, we propose to develop a system that will deploy a swarm of 20 robots in a simulated gazebo environment and help rescue an object successfully to the defined end goal. The robot we propose to use is the Jackal unmanned ground vehicle. Agents in the rescue swarm will spawn at a pre-set start position and then navigate towards the object to be rescued. Once they reach the object contact point the agents will push the object synchronously and start moving towards the goal location.

##### Specifications:
>
>1. Number of agents in swarm : We propose to implement 20 agents and scale it to 30 in final sprint if time permits
>2. Arena : The Gazebo world will will a closed area with wall surroundings and will contain immobile obstacles
>3. Robot : The dimensions of the jackal robot can be seen in the figure [below](https://clearpathrobotics.com/wp-content/uploads/2015/06/front.jpg)
>4. Object : The object to be rescued is a rectangle of dimensions approimately equal to ~ 6m X 6m X 1m
 
#### Jackal Robot interesting facts
| Clearpath Jackal           |          Turtlebot       	  |    
|---------------------------	|----------------------------	|
| 20 Kg payload             	| > 1.3x more than Turtlebot  |
| Run time 40 hrs           	| > 1.6x more than Turtlebot 	|
| Max speed 2m/s            	| > 7.7x more than Turtlebot 	|
| Widely used in industries 	|                            	|

<span align="centre">
<img src="https://clearpathrobotics.com/wp-content/uploads/2015/06/front.jpg" alt="cool Robot"></img>

[source](https://clearpathrobotics.com/wp-content/uploads/2015/06/front.jpg)</span>
- - - -
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
      <a href="https://kavyadevd.github.io/" title="Github" rel="nofollow noreferrer">
      <img src="https://img.icons8.com/ios/50/000000/domain.png" width="19px" alt="github">
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
      <a href="https://aswathselvam.github.io/" title="Github" rel="nofollow noreferrer">
      <img src="https://img.icons8.com/ios/50/000000/domain.png" width="19px" alt="github">
      </a>
      
   </p>
   </div
</div>

- - - -

## UML

### Activity diagram
<p align="centre">
<img src="https://github.com/kavyadevd/swarm_robots/blob/main/UML/ActivityDiagram.png?raw=true" alt="Activity Diagram" width="500px"></img></p>

### Class diagram
<p align="centre">
<img src="https://github.com/kavyadevd/swarm_robots/blob/main/UML/ClassDiagram.png?raw=true" width="500px" alt="Class Diagram"></img>
</p>

### Quad chart
###### The Quad chart is as follows. You can access the pdf version [here](https://github.com/kavyadevd/swarm_robots/blob/main/assets/QuadChart.pdf)
<p align="centre">
<img src="https://github.com/kavyadevd/swarm_robots/blob/main/assets/QuadChart.png" width="500px" alt="QuadChart"></img>
</p>

- - - -


### Presentation
1. Youtube link of our presentation can be found [here](https://youtu.be/-GAqxhDhMFM)
2. The link to the slides can be found [here](https://docs.google.com/presentation/d/1HoAK7hbcWeumYj-4espzwZ5rqsY8IcnH8YzSlovNmkk/edit)

#### Pushing using one jackal and lighter obstacle
https://user-images.githubusercontent.com/13993518/145903641-802c4688-7fb1-414c-a20f-d7afc7ab98a3.mp4

#### Swarm trying to form pattern

https://user-images.githubusercontent.com/13993518/145903837-910c7d92-e646-4126-b9e5-c7e3e306dc72.mp4


### Assumptions:
1. Robots are not aware of its neighbours.
2. Centralized communication, there is a common node(rosmaster) through which all robots communicates with each other. 
3. Swarms only operate in 2D plane.

### Tested on:
1. Ubuntu 20.04 and Ubuntu 18.04
2. OMPL library version: 1.5.2 

- - - -


### Development process
The project will be developed using industry-grade agile methodologies. The agile method being adaptive in nature will quickly adapt to software requirements and changes due to challenges faced if any during the project development cycle. For software development, pair programming strategies will be used which will ensure a robust, bug-free package. We have hierarchical test suites, which will test the submodules by implementing unit test cases and the validity of the whole system. To simulate real-world challenges we plan to use different Gmock test cases and confirm the correctness of the system.

[Sprint planning notes and review](https://docs.google.com/document/d/1nNpMe6DLzv8XDJHyTaXOK77MExHqAslBlWCvYTAc9Zk/edit?usp=sharing)

[AIP Spreadsheet](https://bit.ly/33jHCMR)

### Installations

It is assumed that the system has Ubuntu 18.04 and above with ROS Melodic/Noetic installed.
If not, install Ubuntu from [here](https://ubuntu.com/download/desktop) and ROS from [here](http://wiki.ros.org/melodic/Installation/Ubuntu)

#### First step is to install Jackal robot. Download the setup bash file from [here](https://github.com/kavyadevd/swarm_robots/blob/main/setup.sh).
Execute the downloaded file using following command
```bash
sh setup.sh
```

#### Install dependencies
##### OMPL
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

sudo apt-get update
sudo apt-get install ros-`rosversion -d`-ompl
```

##### octomap_mapping
```bash
sudo apt-get install ros-<ros-version>-octomap ros-<ros-version>-octomap-mapping
rosdep install octomap_mapping
rosmake octomap_mapping

```

#### Clone git repository
```bash
cd swarm_robots/src
git clone --recursive https://github.com/kavyadevd/swarm_robots.git
```

#### Execute program
```bash
catkin_make
roslaunch swarm_robots main.launch
```
On separate terminal run the following command
```
rosrun swarm_robots swarm_robots_node 
```
###### Different options provided with the launch file are:
<ol>
  <li>Launch Rviz : rviz:=y</li>
  <li>Launch rqt topic list : rqt_topic:=y</li>
  <li>Launch rqt graph : rqt_graph:=y</li>
  <li>Launch tf tree : rqt_tf_tree:=y</li>
  <li>Launch rosbag record node : rosbag_yn:=y</li>
</ol> 

###### To launch the demo 
```bash
catkin_make
roslaunch swarm_robots demo.launch
```
On separate terminal run the following command
```
rosrun swarm_robots swarm_robots_node 
```

### Running ROS test/ Gtest

To make the test files execute the following commands successively
```bash
catkin_make tests
rostest swarm_robots test.launch
```

Output will be similiar to :

```bash
... logging to /home/kavya/.ros/log/rostest-Matrix-27255.log
[ROSUNIT] Outputting test results to /home/kavya/.ros/test_results/swarm_robots
/rostest-test_testswarm.xml
[ WARN] [1636828912.923804367]: Publisher message will be changed.
[Testcase: testtestswarm] ... ok

[ROSTEST]-----------------------------------------------------------------------

[swarm_robots.rosunit-testswarm/TestArentsInit][passed]
[swarm_robots.rosunit-testswarm/TestSwarmSize][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/user/.ros/log/rostest-Matrix-27255.log

```

- - - -


## Project dependencies and Licenses
<ol>
  <li>ROS1 Melodic and above (Standard three-clause BSD license)</li>
  <li>ROS packages (all licensed under three-clause BSD license) - roscpp, std_msgs, 
message_generation, actionlib, actionlib_msgs, geometry_msgs, rospy, tf, octomap_msgs, octomap_ros.
  </li>
  <li>The open motion planning library - <a href="https://ompl.kavrakilab.org/license.html">OMPL</a> (Standard three-clause BSD license)</li>
  <li>multi-jackal ROS package forked by <a href="https://github.com/amdpaula/multi_jackal/tree/ros-noetic">amdpaula</a> (Standard three-clause BSD license)</li>
 <li><a href="https://github.com/google/googletest/blob/main/LICENSE">Google Test</a> (Standard three-clause BSD license)</li>
</ol> 

## Licensing
The project is licensed under the [3-Clause BSD License](https://opensource.org/licenses/BSD-3-Clause). Click [here](https://github.com/kavyadevd/swarm_robots/blob/main/LICENSE) to know more.
~~~
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
~~~

- - - -
