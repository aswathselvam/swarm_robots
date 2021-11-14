#include "swarm_robots/agent.h"
#include "swarm_robots/arena.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

void Arena::InitializeAgents() {
    for(int i=0; i<20;i++){
        ros::NodeHandle nh;
        nh.subscribe("/swarm/agent/"+i+"position");
        nh.publisher("/swarm/agent/"+i+"cmd_vel");
        agents.Initiaize(id, nh);
        agents.push_back(Agent());

    }
}