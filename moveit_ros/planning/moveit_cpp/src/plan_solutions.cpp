/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Peter David Fagan
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor te names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Peter David Fagan */

#include <moveit/moveit_cpp/plan_solutions.h>

namespace moveit_cpp
{
/** \brief Function that returns the shortest solution out of a vector of solutions based on robot_trajectory::path_length(...)
 *  \param [in] solutions Vector of solutions to chose the shortest one from
 *  \return Shortest solution, trajectory_ of the returned MotionPlanResponse is a nullptr if no solution is found!
 */
static inline planning_interface::MotionPlanResponse
getShortestSolution(const std::vector<planning_interface::MotionPlanResponse>& solutions)
{
  // Find trajectory with minimal path
  auto const shortest_trajectory = std::min_element(solutions.begin(), solutions.end(),
                                                    [](const planning_interface::MotionPlanResponse& solution_a,
                                                       const planning_interface::MotionPlanResponse& solution_b) {
                                                      // If both solutions were successful, check which path is shorter
                                                      if (solution_a && solution_b)
                                                      {
                                                        return robot_trajectory::path_length(*solution_a.trajectory_) <
                                                               robot_trajectory::path_length(*solution_b.trajectory_);
                                                      }
                                                      // If only solution a is successful, return a
                                                      else if (solution_a)
                                                      {
                                                        return true;
                                                      }
                                                      // Else return solution b, either because it is successful or not
                                                      return false;
                                                    });
  return *shortest_trajectory;
}
}  // namespace moveit_cpp
