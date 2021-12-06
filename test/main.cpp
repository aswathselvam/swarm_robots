/*
 * @file main.cpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @date 05th December 2021
 * @copyright All rights reserved
 * @brief Main cpp file to run test cases
 * Copyright (C) BSD3.
 */

// Include required headers
#include <gtest/gtest.h>
#include <ros/ros.h>

std::shared_ptr<ros::NodeHandle> nh;

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_init");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
