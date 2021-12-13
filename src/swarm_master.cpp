/**
 * @file swarm_master.cpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief File for robot main execution methods
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

#include <string>
#include <vector>

#include "swarm_robots/agent_node.hpp"

using std::cout;
using std::string;
int main(int argc, char** argv) {
    ros::init(argc, argv, "swarm_master");
    ROS_INFO_STREAM("Initialized node swarm_master.");
    vector<AgentNode*> agent_nodes;
    int n = 20;
    try {
        for (int i = 0; i < n; i++) {
            AgentNode* agent_node = new AgentNode(std::to_string(i));
            agent_nodes.push_back(agent_node);
        }

        while (ros::ok()) {
            for (int i = 0; i < n; i++) {
                agent_nodes[i]->Loop();
            }
        }

        return 0;
    } catch (const char* msg) {
        ROS_ERROR_STREAM("ERROR IN MAIN");
        ROS_ERROR_STREAM(msg);
    }
}
