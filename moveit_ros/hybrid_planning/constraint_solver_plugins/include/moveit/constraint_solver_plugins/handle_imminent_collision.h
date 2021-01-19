/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/* Author: Sebastian Jahr
   Description: Simple local solver plugin that stops in front of a collision object.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/local_planner/constraint_solver_interface.h>
#include <control_toolbox/pid.hpp>
//#include <moveit_servo/servo.h>

namespace moveit_hybrid_planning
{
struct PIDConfig
{
  // Default values
  double k_p = 1;
  double k_i = 0;
  double k_d = 0;
  double windup_limit = 0.1;
  double d_t = 0.01;  // s
};

class HandleImminentCollision : public ConstraintSolverInterface
{
public:
  HandleImminentCollision();
  ~HandleImminentCollision() override{};
  bool initialize(const rclcpp::Node::SharedPtr& node,
                  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor) override;

  trajectory_msgs::msg::JointTrajectory
  solve(robot_trajectory::RobotTrajectory local_trajectory,
        std::vector<moveit_msgs::msg::Constraints> local_constraints,
        std::shared_ptr<moveit_msgs::action::LocalPlanner::Feedback> feedback) override;

private:
  rclcpp::Node::SharedPtr node_handle_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  bool feedback_send_;
  std::vector<control_toolbox::Pid> joint_position_pids_;
  PIDConfig pid_config_;
  rclcpp::Rate loop_rate_;
};
}  // namespace moveit_hybrid_planning
