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

#ifndef INCLUDE_SWARM_ROBOTS_SAFETY_CHECK_H_  //  NOLINT
#define INCLUDE_SWARM_ROBOTS_SAFETY_CHECK_H_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

#include <string>
#include <utility>
#include <vector>

#include "state.hpp"

using std::string;

class SafetyCheck {
   public:  //  NOLINT
    /**
     * @brief SafetyCheck parameterized constructor,
     * initializes agent namespace and nodehandle
     * @param string ns : agent namespace
     * @param ros::NodeHandle* nh : ros node handle
     */
    SafetyCheck(string ns, ros::NodeHandle* nh);

    /**
     * @brief Laser scan callback function
     * @param const sensor_msgs::LaserScan::ConstPtr& msg : msg pointer
     */
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    /**
     * @brief Gets safce obstacle free distance
     * @return State : x and y coordinates of safe distance from current
     */
    State GetSafeDirection();

   private:                              //  NOLINT
    double max_range_;                   ///< Range for obstacle avoiance
    double kfov_degrees_;                ///< fov degress = 180
    ros::Subscriber laser_sub_;          ///< Laser topic subscriber
    ros::NodeHandle* nh_;                ///< ros nodehandle
    sensor_msgs::LaserScan laser_scan_;  ///< Variable to store laser scan data
};
#endif  // INCLUDE_SWARM_ROBOTS_SAFETY_CHECK_H_        //  NOLINT
