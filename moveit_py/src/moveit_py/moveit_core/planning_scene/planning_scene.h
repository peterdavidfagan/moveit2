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
#include <copy_ros_msg.h>
#include <serialize_ros_msg.h>
#include <moveit/planning_scene/planning_scene.h>

namespace py = pybind11;

namespace moveit_py
{
namespace bind_planning_scene
{
void apply_planning_scene_world(std::shared_ptr<planning_scene::PlanningScene>& planning_scene,
                                py::object& planning_scene_world_msg);

// void apply_collision_object(std::shared_ptr<planning_scene::PlanningScene>& planning_scene,
//                             py::object& collision_object);

void apply_collision_object(std::shared_ptr<planning_scene::PlanningScene>& planning_scene,
                            py::object& collision_object_msg, std::optional<py::object>& color_msg);

// void apply_collision_object(std::shared_ptr<planning_scene::PlanningScene>& planning_scene, py::object&
// collision_object_msg, py::object& color_msg);

void apply_attached_collision_object(std::shared_ptr<planning_scene::PlanningScene>& planning_scene,
                                     py::object& attached_collision_object);

void apply_octomap(std::shared_ptr<planning_scene::PlanningScene>& planning_scene, py::object& octomap);

Eigen::MatrixXd get_frame_transform(std::shared_ptr<planning_scene::PlanningScene>& planning_scene,
                                    const std::string& id);

py::object get_planning_scene_msg(std::shared_ptr<planning_scene::PlanningScene>& planning_scene);

bool is_path_valid(std::shared_ptr<planning_scene::PlanningScene>& planning_scene,
                   robot_trajectory::RobotTrajectory& robot_trajectory, std::string& group, bool verbose);

bool is_state_colliding(std::shared_ptr<planning_scene::PlanningScene>& planning_scene, std::string group, bool verbose);

bool is_state_colliding(std::shared_ptr<planning_scene::PlanningScene>& planning_scene,
                        const moveit::core::RobotState& robot_state, const std::string& group, bool verbose);

bool is_state_constrained(std::shared_ptr<planning_scene::PlanningScene>& planning_scene,
                          const moveit::core::RobotState& robot_state, py::object constraints, bool verbose);

moveit::core::RobotState& get_current_state(std::shared_ptr<planning_scene::PlanningScene>& planning_scene);

// moveit::core::RobotStatePtr get_current_state_updated(std::shared_ptr<planning_scene::PlanningScene>& planning_scene,
//                                                       py::object update);

void set_current_state(std::shared_ptr<planning_scene::PlanningScene>& planning_scene, py::object& robot_state);

void init_planning_scene(py::module& m);
}  // namespace bind_planning_scene
}  // namespace moveit_py
