#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <iostream>
#include <cmath>
#include "swarm_robots/safety_check.h"
#include "swarm_robots/state.h"

using std::string;
using std::cout;

SafetyCheck::SafetyCheck(string ns, ros::NodeHandle* nh): kfov_degrees_(180){
    this->nh_ = nh;
    
    // Topic name: /jackal0/front/scan
    this->laser_sub_ = this->nh_->subscribe("/jackal"+ns+"/front/scan", 1, &SafetyCheck::ScanCallback, this);
}

void SafetyCheck::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  this->laser_scan_ = *msg;
}


State SafetyCheck::GetSafeDirection(){
    double repel_lateral=0;
    double repel_drive=0;

    //cout<<"Laser min: "<<laser_scan_.range_min;
    //cout<<"Laser max: "<<laser_scan_.range_max;
    //for (int i = 0; i < this->kfov_degrees_ / 2; i++) {
    for(int i= laser_scan_.range_min; i < laser_scan_.range_max; i++){ 
            repel_lateral += (1/laser_scan_.ranges[i])*sin(i * M_PI / 180);
            repel_lateral += (1/laser_scan_.ranges[359-i])*sin(i * M_PI / 180);
            
            repel_drive += 1/laser_scan_.ranges[i]*cos(i * M_PI / 180);
            repel_drive += 1/laser_scan_.ranges[359-i]*cos(i * M_PI / 180);
    }
    
  return State(repel_drive, repel_lateral);
}

