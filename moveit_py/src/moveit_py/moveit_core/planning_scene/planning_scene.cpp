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

void init_planning_scene(py::module& m)
{
  py::class_<planning_scene::PlanningScene, std::shared_ptr<planning_scene::PlanningScene>>(m, "PlanningScene",
                                                                                            R"(
      Representation of the environment as seen by a planning instance. The environment geometry, the robot geometry and state are maintained.
      )")

      // properties
      .def_property("name", &planning_scene::PlanningScene::getName, &planning_scene::PlanningScene::setName,
                    R"(
                    str: The name of the planning scene.
                    )")

      .def_property("robot_model", &planning_scene::PlanningScene::getRobotModel, nullptr,
                    py::return_value_policy::move,
                    R"(
                    :py:class:`moveit_py.core.RobotModel`: The robot model associated to this planning scene.
                    )")

      .def_property("planning_frame", &planning_scene::PlanningScene::getPlanningFrame, nullptr,
                    py::return_value_policy::move,
                    R"(
                    str: The frame in which planning is performed.
                    )")

      .def_property("current_state", &planning_scene::PlanningScene::getCurrentState,
                    &moveit_py::bind_planning_scene::set_current_state, py::return_value_policy::move,
                    R"(
                    :py:class:`moveit_py.core.RobotState`: The current state of the robot.
                    )")

      .def_property("planning_scene_message", &moveit_py::bind_planning_scene::get_planning_scene_msg, nullptr,
                    py::return_value_policy::move)
      // TODO (peterdavidfagan): requires binding of transform object.
      //.def_property("transforms", &planning_scene::PlanningScene::getTransforms, nullptr)

      // methods
      .def("knows_frame_transform",
           py::overload_cast<const moveit::core::RobotState&, const std::string&>(
               &planning_scene::PlanningScene::knowsFrameTransform, py::const_),
           py::arg("robot_state"), py::arg("frame_id"),
           R"(
           Check if a transform to the frame id is known.
           This will be known if id is a link name, an attached body id or a collision object.
           Args:
               robot_state (:py:class:`moveit_py.core.RobotState`): The robot state to check.
               frame_id (str): The frame id to check.
           Returns:
               bool: True if the transform is known, false otherwise.
           )")

      .def("knows_frame_transform",
           py::overload_cast<const std::string&>(&planning_scene::PlanningScene::knowsFrameTransform, py::const_),
           py::arg("frame_id"),
           R"(
           Check if a transform to the frame id is known.
           This will be known if id is a link name, an attached body id or a collision object.
           Args:
               robot_state (:py:class:`moveit_py.core.RobotState`): The robot state to check.
               frame_id (str): The frame id to check.
           Returns:
               bool: True if the transform is known, false otherwise.
           )")

      .def("get_frame_transform", &moveit_py::bind_planning_scene::get_frame_transform, py::arg("frame_id"),
           R"(
           Get the transform corresponding to the frame id. 
           This will be known if id is a link name, an attached body id or a collision object. Return identity when no transform is available.
           Args:
               frame_id (str): The frame id to get the transform for.
           Returns:
               :py:class:`numpy.ndarray`: The transform corresponding to the frame id.
           )")

      //.def("get_current_state_updated", &moveit_py::bind_planning_scene::get_current_state_updated, py::arg("update"),
      //     R"(
      //     Get a copy of the current state with components overwritten by the state message update.
      //
      //     Args:
      //         update (:py:class:`moveit_msgs.msg.RobotState`): The update to apply to the current state.
      //
      //     Returns:
      //         :py:class:`moveit_py.core.RobotState`: The current robot state after applying the update.
      //      )")

      // writing to the planning scene
      .def("apply_planning_scene_world", &moveit_py::bind_planning_scene::apply_planning_scene_world, py::arg("scene"),
           R"(
           Apply a planning scene world message to the current planning scene.
           Args:
               scene (:py:class:`moveit_msgs.msg.PlanningSceneWorld`): The planning scene world message to apply.
       )")

      //.def("apply_collision_object", py::overload_cast<std::shared_ptr<planning_scene::PlanningScene>&,
      // py::object&>(&moveit_py::bind_planning_scene::apply_collision_object), py::arg("object"), R"( Apply
      // a collision object to the planning scene.
      //
      // Args:
      //    object (moveit_msgs.msg.CollisionObject): The collision object to apply to the planning scene.
      //)")

      .def("apply_collision_object", &moveit_py::bind_planning_scene::apply_collision_object,
           py::arg("collision_object_msg"), py::arg("color_msg") = py::none(),
           R"(
           Apply a collision object to the planning scene.
           Args:
               object (moveit_msgs.msg.CollisionObject): The collision object to apply to the planning scene.
           color (moveit_msgs.msg.ObjectColor, optional): The color of the collision object. Defaults to None if not specified.
           )")

      .def("apply_attached_collision_object", &moveit_py::bind_planning_scene::apply_attached_collision_object,
           py::arg("object"),
           R"(
           Apply an attached collision object to the planning scene.
           Args:
               object (moveit_msgs.msg.AttachedCollisionObject): The attached collision object to apply to the planning scene.
           )")

      .def("apply_octomap", &moveit_py::bind_planning_scene::apply_octomap,
           R"(
           Apply an octomap to the planning scene.
           Args:
               octomap (moveit_msgs.msg.Octomap): The octomap to apply to the planning scene.
           )")

      .def("remove_all_collision_objects", &planning_scene::PlanningScene::removeAllCollisionObjects,
           R"(
           Removes collision objects from the planning scene.
	   This method will remove all collision object from the scene except for attached collision objects.
           )")

      // checking state validity
      .def("is_state_colliding",
           py::overload_cast<std::shared_ptr<planning_scene::PlanningScene>&, std::string, bool>(
               &moveit_py::bind_planning_scene::is_state_colliding),
           py::arg("joint_model_group_name"), py::arg("verbose") = false,
           R"(
           Check if the robot state is in collision.
           Args:
               joint_model_group_name (str): The name of the group to check collision for.
               verbose (bool): If true, print the link names of the links in collision.
           Returns:
               bool: True if the robot state is in collision, false otherwise.
           )")

      .def("is_state_colliding",
           py::overload_cast<std::shared_ptr<planning_scene::PlanningScene>&, const moveit::core::RobotState&,
                             const std::string&, bool>(&moveit_py::bind_planning_scene::is_state_colliding),
           py::arg("robot_state"), py::arg("joint_model_group_name"), py::arg("verbose") = false,
           R"(
           Check if the robot state is in collision.
           Args:
               robot_state (:py:class:`moveit_py.core.RobotState`): The robot state to check collision for.
               joint_model_group_name (str): The name of the group to check collision for.
               verbose (bool): If true, print the link names of the links in collision.
           Returns:
               bool: True if the robot state is in collision, false otherwise.
           )")

      .def("is_state_constrained", &moveit_py::bind_planning_scene::is_state_constrained, py::arg("state"),
           py::arg("constraints"), py::arg("verbose") = false,
           R"(
           Check if the robot state fulfills the passed constraints
           Args:
               state (moveit_py.core.RobotState): The robot state to check constraints for.
                   constraints (moveit_msgs.msg.Constraints): The constraints to check.
           verbose (bool): 
           Returns:
               bool: true if state is contrained otherwise false.
           )")

      .def("is_path_valid", &moveit_py::bind_planning_scene::is_path_valid, py::arg("path"),
           py::arg("joint_model_group_name"), py::arg("verbose") = false,
           R"(
           Check if a given path is valid. 
           Each state is checked for validity (collision avoidance and feasibility)
           Args:
               path (:py:class:`moveit_py.core.RobotTrajectory`): The trajectory to check.
               joint_model_group_name (str): The joint model group to check the path against.
               verbose (bool): 
           Returns:
               bool: true if the path is valid otherwise false.
           )")

      // TODO (peterdavidfagan): remove collision result from input parameters and write separate binding code.
      // TODO (peterdavidfagan): consider merging check_collision and check_collision_unpadded into one function with unpadded_param
      .def("check_collision",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&>(
               &planning_scene::PlanningScene::checkCollision),
           py::arg("collision_request"), py::arg("collision_result"),
           R"(
           Check whether the current state is in collision, and if needed, updates the collision transforms of the current state before the computation.
           Args:
               collision_request (:py:class:`moveit_py.core.CollisionRequest`): The collision request to use.
               collision_result (:py:class:`moveit_py.core.CollisionResult`): The collision result to update
           Returns:
               bool: true if state is in collision otherwise false.
           )")

      .def("check_collision",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&,
                             moveit::core::RobotState&>(&planning_scene::PlanningScene::checkCollision, py::const_),
           py::arg("collision_request"), py::arg("collision_result"), py::arg("state"),
           R"(
           Check if the robot state is in collision.
           Args:
               collision_request ():
               collision_result ():
               state ():
           Returns:
               bool: true if state is in collision otherwise false.
           )")

      .def("check_collision",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&,
                             moveit::core::RobotState&, const collision_detection::AllowedCollisionMatrix&>(
               &planning_scene::PlanningScene::checkCollision, py::const_),
           py::arg("collision_request"), py::arg("collision_result"), py::arg("state"), py::arg("acm"),
           R"(
           Check if the robot state is in collision.
           Args:
               collision_request ():
               collision_result ():
               state ():
           acm ():
           Returns:
               bool: true if state is in collision otherwise false.
           )")

      .def("check_collision_unpadded",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&>(
               &planning_scene::PlanningScene::checkCollisionUnpadded),
           py::arg("req"), py::arg("result"),
           R"(
           Check if the robot state is in collision.
           Args:
               collision_request ():
               collision_result ():
           Returns: 
               bool: true if state is in collision otherwise false.
           )")

      .def("check_collision_unpadded",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&,
                             moveit::core::RobotState&>(&planning_scene::PlanningScene::checkCollisionUnpadded,
                                                        py::const_),
           py::arg("collision_request"), py::arg("collision_result"), py::arg("state"),
           R"(
           Check if the robot state is in collision.
           Args:
               collision_request ():
               collision_result ():
               state ():
           Returns:
               bool: true if state is in collision otherwise false.
           )")

      .def("check_collision_unpadded",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&,
                             moveit::core::RobotState&, const collision_detection::AllowedCollisionMatrix&>(
               &planning_scene::PlanningScene::checkCollisionUnpadded, py::const_),
           py::arg("collision_request"), py::arg("collision_result"), py::arg("state"), py::arg("acm"),
           R"(
           Check if the robot state is in collision.
           Args:
               collision_request ():
               collision_result ():
               state ():
           acm ():
           Returns:
               bool: true if state is in collision otherwise false.
           )")

      .def("check_self_collision",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&>(
               &planning_scene::PlanningScene::checkSelfCollision),
           py::arg("collision_request"), py::arg("collision_result"),
           R"(
           Check if the robot state is in collision.
           Args:
               collision_request ():
               collision_result ():
           Returns:
               bool: true if state is in collision otherwise false.
           )")

      .def("check_self_collision",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&,
                             moveit::core::RobotState&>(&planning_scene::PlanningScene::checkSelfCollision, py::const_),
           py::arg("collision_request"), py::arg("collision_result"), py::arg("state"),
           R"(
           Check if the robot state is in collision.
           Args:
               collision request ():
               collision_result ():
               state ():
           Returns:
               bool: true if state is in self collision otherwise false.
           )")

      .def("check_self_collision",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&,
                             moveit::core::RobotState&, const collision_detection::AllowedCollisionMatrix&>(
               &planning_scene::PlanningScene::checkSelfCollision, py::const_),
           py::arg("collision_request"), py::arg("collision_result"), py::arg("state"), py::arg("acm"),
           R"(
           Check if the robot state is in collision.
           Args:
               collision request ():
               collision_result ():
               state ():
           acm():
           Returns:
               bool: true if state is in self collision otherwise false.
           )");
}
}  // namespace bind_planning_scene
}  // namespace moveit_py
