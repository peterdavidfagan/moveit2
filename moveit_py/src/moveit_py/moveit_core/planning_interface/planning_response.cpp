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

#include "planning_response.h"

namespace moveit_py
{
namespace bind_planning_interface
{

std::shared_ptr<robot_trajectory::RobotTrajectory>
get_motion_plan_response_trajectory(std::shared_ptr<planning_interface::MotionPlanResponse>& response)
{
  return response->trajectory_;
}

py::object get_motion_plan_response_start_state(std::shared_ptr<planning_interface::MotionPlanResponse>& response)
{
  py::module_ rclpy_serialization = py::module_::import("rclpy.serialization");
  py::module_ moveit_msgs = py::module_::import("moveit_msgs.msg");

  py::object robot_state = moveit_msgs.attr("RobotState")();
  moveit_msgs::msg::RobotState robot_state_msg = response->start_state_;
  py::bytes serialized_msg = serializeMsg(robot_state_msg);
  rclpy_serialization.attr("deserialize_message")(serialized_msg, robot_state);
  return robot_state;
}

py::object get_motion_plan_response_error_code(std::shared_ptr<planning_interface::MotionPlanResponse>& response)
{
  py::module_ rclpy_serialization = py::module_::import("rclpy.serialization");
  py::module_ moveit_msgs = py::module_::import("moveit_msgs.msg");

  py::object error_code = moveit_msgs.attr("MoveItErrorCodes")();
  moveit_msgs::msg::MoveItErrorCodes error_code_msg =
      static_cast<moveit_msgs::msg::MoveItErrorCodes>(response->error_code_);
  py::bytes serialized_msg = serializeMsg(error_code_msg);
  rclpy_serialization.attr("deserialize_message")(serialized_msg, error_code);
  return error_code;
}

double get_motion_plan_response_planning_time(std::shared_ptr<planning_interface::MotionPlanResponse>& response)
{
  return response->planning_time_;
}

std::string get_motion_plan_response_planner_id(std::shared_ptr<planning_interface::MotionPlanResponse>& response)
{
  return response->planner_id_;
}

void init_motion_plan_response(py::module& m)
{
  py::module planning_interface = m.def_submodule("planning_interface");

  py::class_<planning_interface::MotionPlanResponse, std::shared_ptr<planning_interface::MotionPlanResponse>>(
      planning_interface, "MotionPlanResponse", R"()")

      //.def(py::init<>(), R"()")

      .def_property("trajectory", &moveit_py::bind_planning_interface::get_motion_plan_response_trajectory, nullptr,
                    py::return_value_policy::copy, R"()")

      .def_readonly("planning_time", &planning_interface::MotionPlanResponse::planning_time_,
                    py::return_value_policy::copy, R"()")

      .def_property("error_code", &moveit_py::bind_planning_interface::get_motion_plan_response_error_code, nullptr,
                    py::return_value_policy::copy, R"()")

      .def_property("start_state", &moveit_py::bind_planning_interface::get_motion_plan_response_start_state, nullptr,
                    py::return_value_policy::copy, R"()")

      .def_readonly("planner_id", &planning_interface::MotionPlanResponse::planner_id_, py::return_value_policy::copy,
                    R"()")

      .def("__bool__", [](std::shared_ptr<planning_interface::MotionPlanResponse>& response) {
        return response->error_code_.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      });
}
}  // namespace bind_planning_interface
}  // namespace moveit_py
