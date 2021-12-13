#Clone jackal robot files
sudo apt-get install ros-melodic-jackal-simulator ros-melodic-jackal-desktop
cd ~
mkdir -p swarm_robots/src
cd swarm_robots/src && catkin_init_workspace
git clone https://github.com/jackal/jackal.git
git clone https://github.com/jackal/jackal_simulator.git
git clone https://github.com/clearpathrobotics/LMS1xx.git
git clone https://github.com/ros-drivers/pointgrey_camera_driver.git

#clone multi-jackal repository
git clone https://github.com/kavyadevd/multi_jackal.git

cd ..
catkin_make
