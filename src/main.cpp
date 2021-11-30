#include "swarm_robots/arena.h"

int main(int argc, char **argv) {
  Arena arena;
  arena.InitializeAgents();
  arena.Play();
  return 0;
}