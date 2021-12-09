#include <ros/ros.h>
#include <std_msgs/String.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <message_filters/subscriber.h>
#include <visualization_msgs/Marker.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>

#include "path_planner.h"
#include "State.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using std::string;


//#define DEBUG

#ifdef DEBUG
#define DEBUG_MSG(str) do { std::cout << str << std::endl; } while( false )
#else
#define DEBUG_MSG(str) do { } while ( false )
#endif

PathPlanner::PathPlanner(string ns, ros::NodeHandle* nh){
    this->ns= "_jackal_"+ns;
    this->fixed_frame = "map";
	this->nh_ = nh;
	//ros::Subscriber octree_sub = n.subscribe("/octomap_binary", 1, octomapCallback);
	vis_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker"+this->ns, 0 );
    DEBUG_MSG("OMPL version: " << OMPL_VERSION << std::endl);
}

bool isStateValid(const ob::State *state){

	return true;

	const ob::RealVectorStateSpace::StateType *pos = state->as<ob::RealVectorStateSpace::StateType>();

    // check validity of state defined by pos
	//fcl::Vector3<double> translation(pos->values[0],pos->values[1],pos->values[2]);

	octomap::point3d query(pos->values[0],pos->values[1],0);
  	octomap::OcTreeNode *result = oct_tree_->search(query);
	
	if((result != NULL) && (result->getValue() >= 0.5f)){
		return true;
	}else{
		return false;
	}
}


void PathPlanner::Plan(State start, State goal) {
	// construct the state space we are planning in
	ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));

    // set the bounds for the R^3 part of SE(3)
	space->as<ob::RealVectorStateSpace>()->setBounds(-10, 10);

    // construct an instance of  space information from this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    // set state validity checking for this space
	si->setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));

	// Set our robot's starting state to be the bottom-left corner of
	// the environment, or (0,0).
	ob::ScopedState<> start(space);
	start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_.x;
	start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_.y;
	
	// Set our robot's goal state to be the top-right corner of the
	// environment, or (1,1).
	ob::ScopedState<> goal(space);
	goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_.x;
	goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_.y;

    // create a problem instance
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

    // set the start and goal states
   og::PathGeometric* pth;
	pdef->setStartAndGoalStates(start, goal);

	//pdef->setOptimizationObjective(getPathLengthObjective(si));

    // create a planner for the defined space
	ob::PlannerPtr planner(new og::RRTConnect(si));

    // set the problem we are trying to solve for the planner
	planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
	planner->setup();


    // print the settings for this space
	DEBUG_MSG(si->printSettings(std::cout));
    // print the problem settings
	DEBUG_MSG(pdef->print(std::cout));

    // attempt to solve the problem within one second of planning time
	ob::PlannerStatus solved = planner->solve(0.5);

	if (solved)
	{
		this->success_=true;
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
		std::cout << "Found solution:" << std::endl;
		ob::PathPtr path = pdef->getSolutionPath();
		pth = pdef->getSolutionPath()->as<og::PathGeometric>();
		pth->printAsMatrix(std::cout);
		
		
		//Publish path as markers
		visualization_msgs::Marker path_marker;
		path_marker.action = visualization_msgs::Marker::DELETEALL;
		vis_pub.publish(path_marker);

		for (std::size_t idx = 0; idx < pth->getStateCount (); idx++)
		{
                // cast the abstract state type to the type we expect
			const ob::RealVectorStateSpace::StateType *path_node = pth->getState(idx)->as<ob::RealVectorStateSpace::StateType>();

            // extract the first component of the state and cast it to what we expect
			//const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

            // extract the second component of the state and cast it to what we expect
			//const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

            State node(path_node->values[0], path_node->values[1]);
            this->waypoints_.push_back(node);

			path_marker.header.frame_id = fixed_frame;
			path_marker.header.stamp = ros::Time();
			path_marker.ns = "path"+this->ns;
			path_marker.id = idx;
			path_marker.type = visualization_msgs::Marker::LINE_STRIP;
			path_marker.action = visualization_msgs::Marker::ADD;
			geometry_msgs::Point tmp_p;

			tmp_p.x=path_node->values[0];
			tmp_p.y=path_node->values[1];
			path_marker.points.push_back(tmp_p);
			path_marker.scale.x = 0.1;
			path_marker.scale.y = 0.1;
			path_marker.scale.z = 0.1;
			path_marker.pose.orientation.x = 0;
			path_marker.pose.orientation.y = 0;
			path_marker.pose.orientation.z = 0;
			path_marker.pose.orientation.w = 1;
			path_marker.color.a = 1.0;
			path_marker.color.r = 1.0;
			path_marker.color.g = 1.0;
			path_marker.color.b = 0.0;
			vis_pub.publish(path_marker);
			ros::Duration(0.001).sleep();
		}
        #include <algorithm>
        std::reverse(waypoints_.begin(), waypoints_.end());
		
	}
	else{
        this->success_=false;
		DEBUG_MSG("No solution found" << std::endl);
    }

    return success_;

}


PathPlanner::CreateEmptyMap(){
   oct_tree_ = new octomap::OcTree(0.01);
   oct_tree_.updateNode(endpoint, false);
}


