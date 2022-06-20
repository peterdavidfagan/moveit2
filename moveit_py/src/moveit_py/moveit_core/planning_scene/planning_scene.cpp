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

#include <copy_ros_msg.h>
#include <serialize_ros_msg.h>

#include "planning_scene.h"

namespace moveit_py
{
namespace bind_planning_scene
{

void apply_planning_scene_world(std::shared_ptr<planning_scene::PlanningScene>& planning_scene,
                                py::object& planning_scene_world_msg)
{
  moveit_msgs::msg::PlanningSceneWorld planning_scene_world_cpp;
  py::module_ rclpy_serialization = py::module::import("rclpy.serialization");
  py::bytes serialized_msg = rclpy_serialization.attr("serialize_message")(planning_scene_world_msg);
  deserializeMsg(serialized_msg, planning_scene_world_cpp);
  planning_scene->processPlanningSceneWorldMsg(planning_scene_world_cpp);
}

// void apply_collision_object(std::shared_ptr<planning_scene::PlanningScene>& planning_scene, py::object& collision_object)
//{
//   moveit_msgs::msg::CollisionObject collision_object_cpp = CollisionObjectToCpp(collision_object);
//   planning_scene->processCollisionObjectMsg(collision_object_cpp);
// }

void apply_collision_object(std::shared_ptr<planning_scene::PlanningScene>& planning_scene,
                            py::object& collision_object_msg, std::optional<py::object>& color_msg)
{
  // convert collision object message to cpp equivalent
  moveit_msgs::msg::CollisionObject collision_object_cpp = CollisionObjectToCpp(collision_object_msg);

  // check if color message is provided
  if (color_msg.has_value())
  {
    // convert color message to cpp equivalent (serialization method used for now)
    moveit_msgs::msg::ObjectColor color_cpp;
    py::module_ rclpy_serialization = py::module::import("rclpy.serialization");
    py::bytes serialized_msg = rclpy_serialization.attr("serialize_message")(color_msg);
    deserializeMsg(serialized_msg, color_cpp);

    // apply collision object
    planning_scene->processCollisionObjectMsg(collision_object_cpp);

    // set object color
    planning_scene->setObjectColor(color_cpp.id, color_cpp.color);
  }
  else
  {
    // apply collision object
    planning_scene->processCollisionObjectMsg(collision_object_cpp);
  }
}

void apply_attached_collision_object(std::shared_ptr<planning_scene::PlanningScene>& planning_scene,
                                     py::object& attached_collision_object)
{
  moveit_msgs::msg::AttachedCollisionObject attached_collision_object_cpp;
  py::module_ rclpy_serialization = py::module::import("rclpy.serialization");
  py::bytes serialized_msg = rclpy_serialization.attr("serialize_message")(attached_collision_object);
  deserializeMsg(serialized_msg, attached_collision_object_cpp);
  planning_scene->processAttachedCollisionObjectMsg(attached_collision_object_cpp);
}

void apply_octomap(std::shared_ptr<planning_scene::PlanningScene>& planning_scene, py::object& octomap)
{
  octomap_msgs::msg::Octomap octomap_cpp;
  py::module_ rclpy_serialization = py::module::import("rclpy.serialization");
  py::bytes serialized_msg = rclpy_serialization.attr("serialize_message")(octomap);
  deserializeMsg(serialized_msg, octomap_cpp);
  planning_scene->processOctomapMsg(octomap_cpp);
}

Eigen::MatrixXd get_frame_transform(std::shared_ptr<planning_scene::PlanningScene>& planning_scene,
                                    const std::string& id)
{
  auto transformation = planning_scene->getFrameTransform(id);
  return transformation.matrix();
}

py::object get_planning_scene_msg(std::shared_ptr<planning_scene::PlanningScene>& planning_scene)
{
  moveit_msgs::msg::PlanningScene planning_scene_msg;
  planning_scene->getPlanningSceneMsg(planning_scene_msg);
  py::module_ rclpy_serialization = py::module::import("rclpy.serialization");
  py::bytes serialized_msg = serializeMsg(planning_scene_msg);

  py::module_ moveit_msgs_planning_scene = py::module::import("moveit_msgs.msg._planning_scene");
  py::object planning_scene_msg_py = moveit_msgs_planning_scene.attr("PlanningScene")();
  return rclpy_serialization.attr("deserialize_message")(serialized_msg, planning_scene_msg_py);
}

bool is_path_valid(std::shared_ptr<planning_scene::PlanningScene>& planning_scene,
                   robot_trajectory::RobotTrajectory& robot_trajectory, std::string& group, bool verbose)
{
  return planning_scene->isPathValid(robot_trajectory, group, verbose, nullptr);
}

bool is_state_colliding(std::shared_ptr<planning_scene::PlanningScene>& planning_scene, std::string group, bool verbose)
{
  return planning_scene->isStateColliding(group, verbose);
}

bool is_state_colliding(std::shared_ptr<planning_scene::PlanningScene>& planning_scene,
                        const moveit::core::RobotState& robot_state, const std::string& group, bool verbose)
{
  return planning_scene->isStateColliding(robot_state, group, verbose);
}

bool is_state_constrained(std::shared_ptr<planning_scene::PlanningScene>& planning_scene,
                          const moveit::core::RobotState& robot_state, py::object constraints, bool verbose)
{
  moveit_msgs::msg::Constraints constraints_cpp = ConstraintsToCpp(constraints);
  return planning_scene->isStateConstrained(robot_state, constraints_cpp, verbose);
}

moveit::core::RobotState& get_current_state(std::shared_ptr<planning_scene::PlanningScene>& planning_scene)
{
  return planning_scene->getCurrentStateNonConst();
}

// moveit::core::RobotStatePtr get_current_state_updated(std::shared_ptr<planning_scene::PlanningScene>& planning_scene,
//                                                       py::object update)
//{
//   moveit_msgs::msg::RobotState robot_state_cpp;
//   py::module_ rclpy_serialization = py::module::import("rclpy.serialization");
//   py::bytes serialized_msg = rclpy_serialization.attr("serialize_message")(update);
//   deserializeMsg(serialized_msg, robot_state_cpp);
//
//   return planning_scene->getCurrentStateUpdated(robot_state_cpp);
// }

void set_current_state(std::shared_ptr<planning_scene::PlanningScene>& planning_scene, py::object& robot_state)
{
  // py::module_ moveit_msgs = py::module::import("moveit_msgs.msg");
  // py::object type = moveit_msgs.attr("RobotState")();
  // if (py::isinstance(robot_state, type))
  //{
  //   moveit_msgs::msg::RobotState robot_state_cpp;
  //   py::module_ rclpy_serialization = py::module::import("rclpy.serialization");
  //   py::bytes serialized_msg = rclpy_serialization.attr("serialize_message")(robot_state);
  //   deserializeMsg(serialized_msg, robot_state_cpp);

  //  planning_scene->setCurrentState(robot_state_cpp);
  //}
  // else
  //{
  const moveit::core::RobotState robot_state_cpp = robot_state.cast<const moveit::core::RobotState>();
  planning_scene->setCurrentState(robot_state_cpp);
  //}
}

}  // namespace bind_planning_scene
}  // namespace moveit_py
