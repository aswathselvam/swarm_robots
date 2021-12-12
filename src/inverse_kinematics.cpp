#include <string>
#include <cmath>
#include "swarm_robots/inverse_kinematics.h"
#include "swarm_robots/state.h"

InverseKinematics::InverseKinematics(string ns, ros::NodeHandle* nh){
    safety_check_ = new SafetyCheck(ns, nh);
}

State InverseKinematics::PerformIK(State start, State goal){
    this->current_location_ = start;
    this->goal_location_ = goal;
    State vel =  PerformModelIK();
    State rep_vel = CheckSafety();
    return vel+rep_vel;
}

State InverseKinematics::PerformModelIK(){
    State velocity;
    double dist = sqrt(pow(current_location_.x_ - goal_location_.x_,2) + pow(current_location_.y_ - goal_location_.y_,2));
    double target_yaw = atan2(goal_location_.x_ - current_location_.x_, goal_location_.y_ - current_location_.y_);
    int heading_angle_ = (180/M_PI)* current_location_.yaw_;
    velocity.yaw_ = target_yaw - (heading_angle_-360)%180;
    
    float Kp = 5;
    //Policy 1:
    //velocity_.x=Kp*dist;

    //Policy 2:
    velocity.x_=Kp*abs(dist)/dist;

    return velocity;
}

State InverseKinematics::CheckSafety(){
    safety_check_->GetSafeDirection();
}