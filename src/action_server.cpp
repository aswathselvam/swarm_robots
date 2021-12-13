#include <swarm_robots/swarmACTAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
#include <iostream>

typedef actionlib::SimpleActionServer<swarm_robots::swarmACTAction> Server;

void execute(const swarm_robots::swarmACTGoalConstPtr& goal, Server* as)  
{
  // Do lots of awesome groundbreaking robot stuff here
  std::cout<<goal<<std::endl;
  as->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "do_dishes_server");
  ros::NodeHandle n;
  Server server(n, "do_dishes", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}