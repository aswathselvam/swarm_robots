/**
 * @file main.cpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief main execution file
 * @version 0.2
 * @date 2021-12-11
 * @copyright BSD3 Copyright (c) 2021
 *
 */

#include "swarm_robots/arena.hpp"

int main(int argc, char **argv) {
  Arena arena;
  arena.InitializeAgents();
  arena.Play();
  return 0;
}
