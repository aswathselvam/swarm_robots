#include <cmath>
#include "swarm_robots/inverse_kinematics.h"
void InverseKinematics::PerformIK(){
    PerformModelIK();
    CheckSafety();
}

void InverseKinematics::PerformModelIK(State ){

    double dist = sqrt(pow(position_.x - node.x_,2) + pow(position_.y - node.y_,2));
    double THRESHOLD = 1;
    if(dist<THRESHOLD){
        pathplanner.waypoints.pop_back();
    }
    
    target_yaw = atan2(node.x_ - position_.x, node.y_ - position_.y);
    
    velocity_.yaw = target_yaw - atan2(heading_angle_);
    
    float Kp = 5;
    //Policy 1:
    //velocity_.x=Kp*dist;

    //Policy 2:
    velocity_.x=Kp*abs(dist)/dist;
}

void InverseKinematics::CheckSafety(){
    safety_check_->GetSafeDirection();
}