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
 *   * Neither the name of PickNik Inc. nor the names of its
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

#include <moveit_msgs/msg/robot_state.h>
#include <moveit_msgs/msg/move_it_error_codes.h>
#include <serialize_ros_msg.h>

#include "moveit_cpp.h"

namespace moveit_py
{
namespace bind_moveit_cpp
{

std::shared_ptr<moveit_cpp::PlanningComponent>
get_planning_component(std::shared_ptr<moveit_cpp::MoveItCpp>& moveit_cpp_ptr, std::string planning_component)
{
  return std::make_shared<moveit_cpp::PlanningComponent>(planning_component, moveit_cpp_ptr);
}

py::object get_plan_solution_start_state(std::shared_ptr<moveit_cpp::PlanningComponent::PlanSolution>& plan_solution)
{
  py::module_ rclpy_serialization = py::module::import("rclpy.serialization");
  py::module_ moveit_msgs = py::module::import("moveit_msgs.msg");

  py::object robot_state = moveit_msgs.attr("RobotState")();
  moveit_msgs::msg::RobotState robot_state_cpp = plan_solution->start_state;
  py::bytes serialized_msg = serializeMsg(robot_state_cpp);
  rclpy_serialization.attr("deserialize_message")(serialized_msg, robot_state);
  return robot_state;
}

std::shared_ptr<robot_trajectory::RobotTrajectory> get_plan_solution_trajectory(std::shared_ptr<moveit_cpp::PlanningComponent::PlanSolution>& plan_solution)
{
  return plan_solution->trajectory;
}

py::object get_plan_solution_error_code(std::shared_ptr<moveit_cpp::PlanningComponent::PlanSolution>& plan_solution)
{

  py::module_ rclpy_serialization = py::module::import("rclpy.serialization");
  py::module_ moveit_msgs = py::module::import("moveit_msgs.msg");
  
  py::object moveit_error_codes = moveit_msgs.attr("MoveItErrorCodes")();
  moveit_msgs::msg::MoveItErrorCodes error_codes_cpp = static_cast<moveit_msgs::msg::MoveItErrorCodes>(plan_solution->error_code);
  py::bytes serialized_msg = serializeMsg(error_codes_cpp);
  rclpy_serialization.attr("deserialize_message")(serialized_msg, moveit_error_codes);
  return moveit_error_codes;
}
}  // namespace bind_moveit_cpp
}  // namespace moveit_py
