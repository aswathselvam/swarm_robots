/**
 * @file state.h
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief The header file for state of agents
 * @version 0.1
 * @date 2021-12-05
 * @copyright BSD3 Copyright (c) 2021
 *
 */

#ifndef INCLUDE_SWARM_ROBOTS_STATE_H_
#define INCLUDE_SWARM_ROBOTS_STATE_H_

class State {
 public:
  double x_;
  double y_;
  double yaw_;
  State(double x, double y);
  State(double x, double y, double yaw);
};


#endif  // INCLUDE_SWARM_ROBOTS_STATE_H_
