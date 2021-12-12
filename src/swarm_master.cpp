/**
 * @file swarm_master.cpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief File for robot main execution methods
 * @version 0.2
 * @date 2021-12-11
 * @copyright BSD3 Copyright (c) 2021
 *
 */

#include <string>
#include <vector>

#include "../include/swarm_robots/agent_node.hpp"

using std::string;
using std::vector;

int main(int argc, char **argv) {
    // std::string ns = "";
    // AgentNode agent_node(ns);
    // //vector<AgentNode> agent_nodes;
    // for (int i = 0; i < 2; i++) {
    //   //AgentNode agent_node(std::to_string(i));
    //   //agent_nodes.push_back(agent_node);
    // }

    ros::init(argc, argv, "swarm_control");
    ros::NodeHandle n;

    geometry_msgs::Twist robot_vel;  // Robot twist message object
    ros::Rate publish_rate(10);
    ros::Publisher send_velocity =
        n.advertise<geometry_msgs::Twist>
        ("/jackal2/jackal_velocity_controller/cmd_vel", 1);

    while (ros::ok()) {
        // Arena arena_(n);
        // State agent1_state;
        std::string s = "jackal0";
        // PathPlanner planner(s, n);
        // agent1_state = planner.GetContactPoint(1);

        robot_vel.linear.x = 0.3;
        robot_vel.linear.y = 0.0;
        robot_vel.linear.z = 0.0;
        robot_vel.angular.x = 0.0;
        robot_vel.angular.y = 0.0;
        robot_vel.angular.z = 0.0;
        send_velocity.publish(robot_vel);
        ros::spinOnce();
        publish_rate.sleep();
        ROS_INFO_STREAM("Velocity:" << robot_vel.linear.x);
        ROS_INFO_STREAM(" and " << robot_vel.angular.z);
    }

    return 0;
}
