/**
 * @file safety_check.cpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief File for robot agent safety check methods
 * @version 0.2
 * @date 2021-12-11
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
#include "swarm_robots/safety_check.hpp"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <cmath>
#include <iostream>
#include <string>

#include "swarm_robots/state.hpp"

using std::cout;
using std::string;

SafetyCheck::SafetyCheck(string ns, ros::NodeHandlePtr nh) : kfov_degrees_(180) {
    this->nh_ = nh;

    // Topic name: /jackal0/front/scan
    this->laser_sub_ =
    this->nh_->subscribe("/jackal" + ns + "/front/scan", 1,
    &SafetyCheck::ScanCallback, this);
}

void SafetyCheck::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    this->laser_scan_ = *msg;
}

State SafetyCheck::GetSafeDirection() {
    double repel_lateral = 0;
    double repel_drive = 0;

    // cout<<"Laser min: "<<laser_scan_.range_min;
    // cout<<"Laser max: "<<laser_scan_.range_max;
    // for (int i = 0; i < this->kfov_degrees_ / 2; i++) {
    /*
    for (int i = laser_scan_.range_min; i < laser_scan_.range_max; i++) {
        repel_lateral += (1 / laser_scan_.ranges[i]) * sin(i * M_PI / 180);
        repel_lateral +=
        (1 / laser_scan_.ranges[359 - i]) * sin(i * M_PI / 180);

        repel_drive += 1 / laser_scan_.ranges[i] * cos(i * M_PI / 180);
        repel_drive += 1 / laser_scan_.ranges[359 - i] * cos(i * M_PI / 180);
    }
    */

    return State(repel_drive, repel_lateral);
}
