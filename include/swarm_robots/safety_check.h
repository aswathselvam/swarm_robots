/**
 * @file safety_check.h
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief The header file for agent robot safety check
 * @version 0.1
 * @date 2021-12-05
 * @copyright BSD3 Copyright (c) 2021
 *
 */


#ifndef INCLUDE_SWARM_ROBOTS_SAFETY_CHECK_H_        //  NOLINT
#define INCLUDE_SWARM_ROBOTS_SAFETY_CHECK_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <utility>
#include <vector>
#include <string>

using std::string;

class SafetyCheck {
    public:        //  NOLINT
        SafetyCheck(string ns, ros::NodeHandle* nh);
        State GetSafeDirection();

    private:        //  NOLINT
        double max_range_;
        const double kfov_degrees_;
        ros::Subscriber laser_sub_;
        ros::NodeHandle nh_;
        sensor_msgs::LaserScan laser_scan_;
};
#endif  // INCLUDE_SWARM_ROBOTS_SAFETY_CHECK_H_        //  NOLINT
