/**
 * @file arena.hpp
 * @author Kavyashree Devadiga (kavya@umd.edu), Aswath Muthuselvam
 * (aswath@umd.edu)
 * @brief The header file for arena elements
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
#ifndef INCLUDE_SWARM_ROBOTS_ARENA_HPP_  //  NOLINT
#define INCLUDE_SWARM_ROBOTS_ARENA_HPP_

#include <std_msgs/String.h>

#include <vector>

#include "agent.hpp"
#include "state.hpp"

using std::vector;
class Arena {
   public:  //  NOLINT
    /**
     * @brief Inilializes all agents in arena
     * @return bool: True if successful
     */
    bool InitializeAgents();

    // Method Not needed for current flow
    // State GetOrigin();
    // std::vector<std::vector<double>> GetBoundary();
    // void Play();

    /**
     * @brief Returns total active agents in swarm
     * @return int: swarm size
     */
    int GetSwarmSize();

   private:                       //  NOLINT
    const int total_agents = 20;  ///< Number of agents in arena
    vector<Agent> agents;         ///< Vector of all agent objects in arena
    // vector<double, double> boundary;
    // State origin;
};

#endif  // INCLUDE_SWARM_ROBOTS_ARENA_HPP_        //  NOLINT
