
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
#include "std_msgs/String.h"

#include "swarm_robots/agent.h"
#include "swarm_robots/arena.h"

/**
 * @brief Test if Initialization is successful
 */
TEST(TestStub1, TestPublisherExists) {
  ros::NodeHandle nh;
  Arena arena;
  EXPECT_TRUE(arena.InitializeAgents());
}

/**
 * @brief Tests total agents spawned
 */
TEST(TestStub2, TestMessageChangeService) {
  Arena arena;
  EXPECT_EQ(arena.GetSwarmSize(), 4);
}
