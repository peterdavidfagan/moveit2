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

#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <copy_ros_msg.h>
#include <serialize_ros_msg.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include "moveit_cpp.h"
#include "../planning_scene_monitor/planning_scene_monitor.h"

namespace py = pybind11;

namespace moveit_py
{
namespace bind_planning_component
{
bool set_goal(std::shared_ptr<moveit_cpp::PlanningComponent>& planning_components, py::object& pose_stamped,
              std::string link_name);

bool set_goal(std::shared_ptr<moveit_cpp::PlanningComponent>& planning_component, py::list& constraints);

bool set_goal(std::shared_ptr<moveit_cpp::PlanningComponent>& planning_component, py::array_t<double> pose_goal,
              std::string& link_name);

bool set_path_constraints(std::shared_ptr<moveit_cpp::PlanningComponent>& planning_component,
                          py::object path_constraints);

void init_plan_request_parameters(py::module &m);

void init_plan_solution(py::module &m);

void init_planning_component_context_manager(py::module &m);

void init_planning_component(py::module &m);
}  // namespace bind_planning_component
}  // namespace moveit_py