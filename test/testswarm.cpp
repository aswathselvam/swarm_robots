
/**
 * @file testswarm.cpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @date 13th November 2021
 * @brief Cpp file to define test cases
 * @copyright BSD3 Copyright (c) 2021
 */

// Include required headers
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>

#include "../include/swarm_robots/agent.hpp"
#include "../include/swarm_robots/arena.hpp"
#include "../include/swarm_robots/forward_kinematics.hpp"
#include "../include/swarm_robots/obstacle.hpp"
#include "../include/swarm_robots/safety_check.hpp"
#include "../include/swarm_robots/state.hpp"
#include "std_msgs/String.h"
#include "swarm_robots/agent_node.hpp"

ros::NodeHandlePtr nhp;

/**
 * @brief Test if Initialization is successful
 */
TEST(TestStub1, TestArentsInit) {
    vector<AgentNode*> agent_nodes;
    int n = 5;
    for (int i = 0; i < n; i++) {
        AgentNode* agent_node = new AgentNode(std::to_string(i));
        agent_nodes.push_back(agent_node);
    }
    bool flag = true;
    for (int i = 0; i < n; i++) {
        if (agent_nodes[i] == NULL) flag = false;
    }
    EXPECT_TRUE(flag);
}

/**
 * @brief Tests Agent class initialization
 */
TEST(TestStub2, TestAgentClassInit) {
    Agent agent_("2");
    EXPECT_EQ(agent_.heading_angle_, 180);
}

/**
 * @brief Tests total agents spawned
 */
TEST(TestStub3, TestSwarmSize) {
    Arena arena;
    EXPECT_EQ(arena.GetSwarmSize(), 20);
}

/**
 * @brief Test FK Move forward method
 */
TEST(TestStub4, TestMoveForward) {
    geometry_msgs::Twist robot_vel;  // Robot twist message object
    ForwardKinematics FK;
    FK.MoveForward(&robot_vel);
    EXPECT_GT(robot_vel.linear.x, 0);
}

/**
 * @brief Test FK Stop method
 */
TEST(TestStub5, TestFKStop) {
    geometry_msgs::Twist robot_vel;  // Robot twist message object
    ForwardKinematics FK;
    FK.Stop(&robot_vel);
    EXPECT_EQ(robot_vel.linear.x, 0);
}

/**
 * @brief Test Obstacle constructor
 */
TEST(TestStub6, TestObstacleInit1) {
    Obstacle obj("2");
    EXPECT_NE(obj.length_, 0);
}

/**
 * @brief Test Obstacle constructor
 */
TEST(TestStub7, TestObstacleInit2) {
    Obstacle obj("");
    EXPECT_NE(obj.length_, 1);
}

/**
 * @brief Test State class default constructor
 */
TEST(TestStub8, TestStateInit1) {
    State s;
    EXPECT_EQ(s.x_, 0);
}

/**
 * @brief Test State class constructor with two param
 */
TEST(TestStub9, TestStateInit2) {
    State s(2, 3);
    EXPECT_EQ(s.x_, 2);
    EXPECT_EQ(s.yaw_, 0);
}

/**
 * @brief Test State class constructor with three param
 */
TEST(TestStub10, TestStateInit3) {
    State s(2, 3, 11);
    EXPECT_EQ(s.x_, 2);
    EXPECT_EQ(s.yaw_, 11);
}
