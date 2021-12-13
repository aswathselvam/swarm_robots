/**
 * @file agent.cpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief The code file for agent control
 * @version 0.1
 * @date 2021-12-05
 * @copyright BSD3 Copyright (c) 2021
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED 
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 *  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "swarm_robots/agent.hpp"

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <cmath>

#include "swarm_robots/arena.hpp"
#include "swarm_robots/forward_kinematics.hpp"
#include "swarm_robots/inverse_kinematics.hpp"
#include "swarm_robots/path_planner.hpp"
#include "swarm_robots/safety_check.hpp"
#include "swarm_robots/state.hpp"

using std::string;

Agent::Agent(string agent_id) {
    this->agent_id_ = agent_id;
    this->position_.x_ = 0;
    this->position_.y_ = 0;
    this->position_.yaw_ = 0;
    this->velocity_.x_ = 0;
    this->velocity_.y_ = 0;
    this->velocity_.yaw_ = 0;
    this->heading_angle_ = 180;
    this->forward_kinematics_= NULL;
    this->inverse_kinematics_ = NULL;
}

void Agent::PlanPath() {
    // path_planner_->Plan(this->position_, this->goal_pos_);
}

void Agent::PerformInverseKinematics() {
    /*
      if(!path_planner_->waypoints_.size()>0)
        PlanPath();
    
    State intermediate_goal = *( path_planner_->waypoints_.begin() );
    */

    //Provide a Sample goal state: 
    //State intermediate_goal(2, 4);
    
    State intermediate_goal = path_planner_->GetContactPoint(std::stoi(agent_id_));
    this->velocity_ =
        inverse_kinematics_->PerformIK( this->position_, intermediate_goal);

    /*
    double dist = sqrt(pow(intermediate_goal.x_ - this->position_.x_,2) + pow(intermediate_goal.y_ - this->position_.y_,2));
    double THRESHOLD = 1;
    if(dist<THRESHOLD){
        path_planner_->waypoints_.pop_back();
    }
    */
}

void Agent::PerformForwardKinematics() {
    this->velocity_ = forward_kinematics_->PerformFK(this->velocity_);
}

void Agent::Stop() {
    velocity_.x_ = 0;
    velocity_.y_ = 0;
    velocity_.yaw_ = 0;
}
