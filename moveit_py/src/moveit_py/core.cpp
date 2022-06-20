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

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <rclcpp/rclcpp.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <srdfdom/srdf_writer.h>
#include <fstream>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <copy_ros_msg.h>
#include <serialize_ros_msg.h>

#include "moveit_core/robot_state/robot_state.h"
#include "moveit_core/planning_scene/planning_scene.h"

PYBIND11_MODULE(core, m)
{
  m.doc() = R"(
            Python bindings for moveit_core functionalities.
            )";

  // Provide custom function signatures
  py::options options;
  options.disable_function_signatures();

  // TODO (peterdavidfagan): complete LinkModel bindings
  // LinkModel
  // py::class_<moveit::core::LinkModel>(m, "LinkModel");

  // TODO (peterdavidfagan): complete JointModel bindings
  // JointModel (this is an abstract base class)
  // py::class_<moveit::core::JointModel>(m, "JointModel");

  // JointModelGroup (no direct initialization for now)
  py::class_<moveit::core::JointModelGroup>(m, "JointModelGroup",
                                            R"(
          Representation of a group of joints that are part of a robot model.
          )")

      .def_property("name", &moveit::core::JointModelGroup::getName, nullptr,
                    R"(
                    str: The name of the joint model group.
                    )")

      .def_property("joint_model_names", &moveit::core::JointModelGroup::getJointModelNames, nullptr,
                    R"(
                    list of str: The names of the joint models in the group.
                    )");

  // RobotModel
  py::class_<moveit::core::RobotModel, std::shared_ptr<moveit::core::RobotModel>>(m, "RobotModel",
                                                                                  R"(
          Representation of a kinematic model.
          )")

      .def(py::init([](std::string& urdf_xml_path, std::string& srdf_xml_path) {
             // Read in URDF
             std::string xml_string;
             std::fstream xml_file(urdf_xml_path.c_str(), std::fstream::in);
             while (xml_file.good())
             {
               std::string line;
               std::getline(xml_file, line);
               xml_string += (line + "\n");
             }
             xml_file.close();

             urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(xml_string);

             // Read in SRDF
             srdf::Model srdf_model;
             srdf_model.initFile(*urdf_model, srdf_xml_path.c_str());

             // TODO (peterdavidfagan): revise the below.
             srdf::ModelConstSharedPtr srdf_model_;
             srdf_model_ = std::make_shared<const srdf::Model>(std::move(srdf_model));

             // Instantiate robot model
             return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model_);
           }),
           py::arg("urdf_xml_path"), py::arg("srdf_xml_path"),
           R"(
           Initializes a kinematic model for a robot.
           
           Args:
               urdf_xml_path (str): The filepath to the urdf file describing the robot.
               srdf_xml_path (str): The filepath to the srdf file describing the robot semantics.

           Returns:
               moveit_py.core.RobotModel: The kinematic model for the robot.
       )")

      .def_property("name", &moveit::core::RobotModel::getName, nullptr,
                    R"(
                    str: The name of the robot model.
                    )")

      .def_property("model_frame", &moveit::core::RobotModel::getModelFrame, nullptr,
                    R"( 
                    str: Get the frame in which the transforms for this model are computed (when using a :py:class:`moveit_py.core.RobotState`). 

                    This frame depends on the root joint. As such, the frame is either extracted from SRDF, or it is assumed to be the name of the root link.
                   )")

      // TODO (peterdavidfagan): make available when JointModel is binded
      //.def_property("root_joint", &moveit::core::RobotModel::getRootJoint, nullptr, "The root joint of the robot model.")

      .def_property("root_joint_name", &moveit::core::RobotModel::getRootJointName, nullptr,
                    R"(
                    str: The name of the root joint.
                    )")

      .def(
          "get_model_info",
          [](std::shared_ptr<moveit::core::RobotModel>& s) {
            std::stringstream ss;
            s->printModelInfo(ss);
            return ss.str();
          },
          py::return_value_policy::move,
          R"( 
          Gets a formatted string containing a summary of relevant information from the robot model.          

          Returns:
              str: Formatted string containing generic robot model information.
          )")

      // Interacting with joint model groups
      .def_property("joint_model_group_names", &moveit::core::RobotModel::getJointModelGroupNames, nullptr,
                    R"(
                    list of str: The names of the joint model groups in the robot model.
                    )")

      .def_property("joint_model_groups", py::overload_cast<>(&moveit::core::RobotModel::getJointModelGroups), nullptr,
                    py::return_value_policy::reference_internal,
                    R"(
                    list of moveit_py.core.JointModelGroup: The joint model groups available in the robot model.
                    )")

      .def("has_joint_model_group", &moveit::core::RobotModel::hasJointModelGroup, py::arg("joint_model_group_name"),
           R"(
           Checks if a joint model group with the given name exists in the robot model.

           Returns:
                   bool: true if joint model group exists.
           )")

      .def("get_joint_model_group",
           py::overload_cast<const std::string&>(&moveit::core::RobotModel::getJointModelGroup),
           py::arg("joint_model_group_name"), py::return_value_policy::reference_internal,
           R"(
           Gets a joint model group instance by name.

           Args:
               joint_model_group_name (str): The name of the joint model group to return.

           Returns:
               :py:class:`moveit_py.core.JointModelGroup`: joint model group instance that corresponds with joint_model_group_name parameter.
           )");

  // RobotState
  py::class_<moveit::core::RobotState, std::shared_ptr<moveit::core::RobotState>>(m, "RobotState",
                                                                                  R"(
          Representation of a robot's state.

          At the lowest level, a state is a collection of variables. Each variable has a name and can have position, velocity, acceleration and effort associated to it. Effort and acceleration share the memory area for efficiency reasons (one should not set both acceleration and effort in the same state and expect things to work). Often variables correspond to joint names as well (joints with one degree of freedom have one variable), but joints with multiple degrees of freedom have more variables. Operations are allowed at variable level, joint level (see JointModel) and joint group level (see JointModelGroup).

                  For efficiency reasons a state computes forward kinematics in a lazy fashion. This can sometimes lead to problems if the update() function was not called on the state.
          )")

      .def(py::init<const std::shared_ptr<const moveit::core::RobotModel>&>(),
           R"(
           Initializes robot state from a robot model.
               
           Args:
               :py:class:`moveit_py.core.RobotModel`: The robot model associated to the instantiated robot state.
               
           )")

      // Get underlying robot model, frame transformations and jacobian
      .def_property("robot_model", &moveit::core::RobotState::getRobotModel, nullptr,
                    py::return_value_policy::reference,
                    R"(
                    :py:class:`moveit_py.core.RobotModel`: The robot model instance associated to this robot state.
                    )")

      .def_property("dirty", &moveit::core::RobotState::dirty, nullptr,
                    R"(
            bool: True if the robot state is dirty.
            )")

      .def("get_frame_transform", &moveit_py::bind_robot_state::get_frame_transform, py::arg("frame_id"),
           py::return_value_policy::move,
           R"(
           Get the transformation matrix from the model frame (root of model) to the frame identified by frame_id.

           If frame_id was not found, frame_found is set to false and an identity transform is returned.

       This method is restricted to frames defined within the robot state and doesn't include collision object present in the collision world. Please use the PlanningScene.get_frame_transform method for collision world objects. 

           Args:
               frame_id (str): The id of the frame to get the transform for.

           Returns:
               :py:class:`numpy.ndarray`: The transformation matrix from the model frame to the frame identified by frame_id.
           )")

      .def("get_pose", &moveit_py::bind_robot_state::get_pose, py::arg("link_name"),
           R"(
           Get the pose of a link that is defined in the robot model.

           Args:
               link_name (str): The name of the link to get the pose for.

           Returns:
               geometry_msgs.msg.Pose: A ROS geometry message containing the pose of the link.
           )")

      .def("get_jacobian",
           py::overload_cast<std::shared_ptr<moveit::core::RobotState>&, const std::string&, const Eigen::Vector3d&>(
               &moveit_py::bind_robot_state::get_jacobian),
           py::arg("joint_model_group_name"), py::arg("reference_point_position"), py::return_value_policy::move,
           R"(
           Compute the Jacobian with reference to the last link of a specified group.

           Args:
               joint_model_group_name (str): The name of the joint model group to compute the Jacobian for.
               reference_point_position (:py:class:`numpy.ndarray`): The position of the reference point in the link frame.

           Returns:
               :py:class:`numpy.ndarray`: The Jacobian of the specified group with respect to the reference point.

           Raises:
               Exception: If the group is not a chain.
           )")

      .def("get_jacobian",
           py::overload_cast<std::shared_ptr<moveit::core::RobotState>&, const std::string&, const std::string,
                             const Eigen::Vector3d&, bool>(&moveit_py::bind_robot_state::get_jacobian),
           py::arg("joint_model_group_name"), py::arg("link_name"), py::arg("reference_point_position"),
           py::arg("use_quaternion_representation") = false, py::return_value_policy::move,
           R"(
           Compute the Jacobian with reference to a particular point on a given link, for a specified group.

           Args:
               joint_model_group_name (str): The name of the joint model group to compute the Jacobian for.
               link_name (str): The name of the link model to compute the Jacobian for.
               reference_point_position (:py:class:`numpy.ndarray`): The position of the reference point in the link frame.
           use_quaternion_representation (bool): If true, the Jacobian will be represented as a quaternion.

           Returns:
               :py:class:`numpy.ndarray`: The Jacobian of the specified group with respect to the reference point.
           )")

      // Get state information
      .def_property("state_tree", py::overload_cast<>(&moveit::core::RobotState::getStateTreeString, py::const_),
                    nullptr, py::return_value_policy::move,
                    R"(
                    str: represents the state tree of the robot state.
                    )")

      // TODO (peterdavidfagan): move to property.
      .def(
          "get_state_info",
          [](std::shared_ptr<moveit::core::RobotState>& s) {
            std::stringstream ss;
            s->printStateInfo(ss);
            return ss.str();
          },
          py::return_value_policy::move,
          R"(
          Returns:
              str: represents the current state information.
          )")

      // Getting and setting joint model group positions, velocities, accelerations
      .def("set_joint_group_positions",
           py::overload_cast<const std::string&, const Eigen::VectorXd&>(
               &moveit::core::RobotState::setJointGroupPositions),
           py::arg("joint_model_group_name"), py::arg("position_values"),
           R"(
           Sets the positions of the joints in the specified joint model group.

           Args: 
               joint_model_group_name (str):
               position_values (:py:class:`numpy.ndarray`): The positions of the joints in the joint model group.
       )")

      // peterdavidfagan: I am not sure if additional function names are better than having function parameters for joint setting.
      .def("set_joint_group_active_positions",
           py::overload_cast<const std::string&, const Eigen::VectorXd&>(
               &moveit::core::RobotState::setJointGroupActivePositions),
           py::arg("joint_model_group_name"), py::arg("position_values"),
           R"(
           Sets the active positions of joints in the specified joint model group.
           
           Args: 
               joint_model_group_name (str): The name of the joint model group to set the active positions for.
               position_values (:py:class:`numpy.ndarray`): The positions of the joints in the joint model group.
       )")

      .def("get_joint_group_positions", &moveit_py::bind_robot_state::copy_joint_group_positions,
           py::arg("joint_model_group_name"),
           R"(
           For a given group, get the position values of the variables that make up the group.

           Args:
               joint_model_group_name (str): The name of the joint model group to copy the positions for.

           Returns:
               :py:class:`numpy.ndarray`: The positions of the joints in the joint model group.
           )")

      .def("set_joint_group_velocities",
           py::overload_cast<const std::string&, const Eigen::VectorXd&>(
               &moveit::core::RobotState::setJointGroupVelocities),
           py::arg("joint_model_group_name"), py::arg("velocity_values"),
           R"(
           Sets the velocities of the joints in the specified joint model group.

           Args:
               joint_model_group_name (str): The name of the joint model group to set the velocities for.
               velocity_values (:py:class:`numpy.ndarray`): The velocities of the joints in the joint model group.
           )")

      .def("get_joint_group_velocities", &moveit_py::bind_robot_state::copy_joint_group_velocities,
           py::arg("joint_model_group_name"),
           R"(
           For a given group, get the velocity values of the variables that make up the group.
           
           Args:
               joint_model_group_name (str): The name of the joint model group to copy the velocities for.

           Returns:
               :py:class:`numpy.ndarray`: The velocities of the joints in the joint model group.   
       )")

      .def("set_joint_group_accelerations",
           py::overload_cast<const std::string&, const Eigen::VectorXd&>(
               &moveit::core::RobotState::setJointGroupAccelerations),
           py::arg("joint_model_group_name"), py::arg("acceleration_values"),
           R"(
           Sets the accelerations of the joints in the specified joint model group.
               
           Args:
               joint_model_group_name (str): The name of the joint model group to set the accelerations for.
               acceleration_values (:py:class:`numpy.ndarray`): The accelerations of the joints in the joint model group.
           )")

      .def("get_joint_group_accelerations", &moveit_py::bind_robot_state::copy_joint_group_accelerations,
           py::arg("joint_model_group_name"),
           R"(
           For a given group, get the acceleration values of the variables that make up the group.

           Args:
               joint_model_group_name (str): The name of the joint model group to copy the accelerations for.

           Returns:
               :py:class:`numpy.ndarray`: The accelerations of the joints in the joint model group.
           )")

      // Forward kinematics
      .def("get_global_link_transform", &moveit_py::bind_robot_state::get_global_link_transform, py::arg("link_name"),
           R"(
       Returns the transform of the specified link in the global frame.

       Args:
           link_name (str): The name of the link to get the transform for.

       Returns:
           :py:class:`numpy.ndarray`: The transform of the specified link in the global frame.
       )")

      // Setting state from inverse kinematics
      .def("set_from_ik", &moveit_py::bind_robot_state::set_from_ik, py::arg("joint_model_group_name"),
           py::arg("geometry_pose"), py::arg("tip_name"), py::arg("timeout") = 0.0,
           R"(
           Sets the state of the robot to the one that results from solving the inverse kinematics for the specified group.

           Args:
               joint_model_group_name (str): The name of the joint model group to set the state for.
               geometry_pose (geometry_msgs.msg.Pose): The pose of the end-effector to solve the inverse kinematics for.
               tip_name (str): The name of the link that is the tip of the end-effector.
               timeout (float): The amount of time to wait for the IK solution to be found.
           )")

      // Setting entire state values
      .def("set_to_default_values", py::overload_cast<>(&moveit::core::RobotState::setToDefaultValues),
           R"(
           Set all joints to their default positions. 

           The default position is 0, or if that is not within bounds then half way between min and max bound.
           )")

      .def("set_to_default_values",
           py::overload_cast<const moveit::core::JointModelGroup*, const std::string&>(
               &moveit::core::RobotState::setToDefaultValues),
           py::arg("joint_model_group"), py::arg("name"),
           R"(
           Set the joints in group to the position name defined in the SRDF.

           Args:
               joint_model_group (:py:class:`moveit_py.core.JointModelGroup`): The joint model group to set the default values for.
               name (str): The name of a predefined state which is defined in the robot model SRDF.
           )")

      .def("set_to_default_values",
           py::overload_cast<std::shared_ptr<moveit::core::RobotState>&, const std::string&, const std::string&>(
               &moveit_py::bind_robot_state::set_to_default_values),
           py::arg("joint_model_group_name"), py::arg("name"),
           R"(
           Set the joints in group to the position name defined in the SRDF.

           Args:
               joint_model_group_name (str): The name of the joint model group to set the default values for.
               name (str): The name of a predefined state which is defined in the robot model SRDF.
       )")

      .def("set_to_random_positions", py::overload_cast<>(&moveit::core::RobotState::setToRandomPositions),
           R"(
           Set all joints to random positions within the default bounds.
           )")

      .def("set_to_random_positions",
           py::overload_cast<const moveit::core::JointModelGroup*>(&moveit::core::RobotState::setToRandomPositions),
           py::arg("joint_model_group"),
           R"(
           Set all joints in the joint model group to random positions within the default bounds.

           Args:
               joint_model_group (:py:class:`moveit_py.core.JointModelGroup`): The joint model group to set the random values for.
           )")

      .def("clear_attached_bodies", py::overload_cast<>(&moveit::core::RobotState::clearAttachedBodies),
           R"(
      	   Clear all attached bodies.
      
      	   We only allow for attaching of objects via the PlanningScene instance. This method allows any attached objects that are associated to this RobotState instance to be removed. 
	   )")

      .def("update", &moveit_py::bind_robot_state::update, py::arg("force") = false, py::arg("type") = "all",
           R"(
             Update state transforms.
  
             Args:
                 force (bool):
             category (str): specifies the category to update. All indicates updating all transforms while "links_only"
             and "collisions_only" ensure that only links or collision transforms are updated. )");

  // RobotTrajectory
  py::class_<robot_trajectory::RobotTrajectory, std::shared_ptr<robot_trajectory::RobotTrajectory>>(m,
                                                                                                    "RobotTrajectory",
                                                                                                    R"(
                                                    Maintains a sequence of waypoints and the durations between these waypoints.
                                                    )")

      .def("__getitem__", &robot_trajectory::RobotTrajectory::getWayPoint, py::arg("idx"),
           R"(
           Get the waypoint at the specified index in the trajectory.
           
           Returns:
               :py:class:`moveit_py.core.RobotState`: The robot state corresponding to a waypoint at the specified index in the trajectory.
           )")

      .def("__len__", &robot_trajectory::RobotTrajectory::getWayPointCount,
           R"(
                    Returns:
                        int: The number of waypoints in the trajectory.
                    )")

      .def("__reverse__", &robot_trajectory::RobotTrajectory::reverse,
           R"(
     	   Reverse the trajectory.
     	   )")

      .def_property("joint_model_group_name", &robot_trajectory::RobotTrajectory::getGroupName,
                    &robot_trajectory::RobotTrajectory::setGroupName,
                    R"(
                    str: The name of the joint model group that this trajectory is for.
                    )")

      .def_property("robot_model", &robot_trajectory::RobotTrajectory::getRobotModel, nullptr,
                    R"(
                    :py:class:`moveit_py.core.RobotModel`: The robot model that this trajectory is for.
                    )")

      .def_property("duration", &robot_trajectory::RobotTrajectory::getDuration, nullptr,
                    R"(
                    float: The duration of the trajectory.
                    )")

      .def_property("average_segment_duration", &robot_trajectory::RobotTrajectory::getAverageSegmentDuration, nullptr,
                    R"(
                    float: The average duration of the segments in the trajectory.
                    )")

      .def("unwind", py::overload_cast<>(&robot_trajectory::RobotTrajectory::unwind),
           R"(
           Unwind the trajectory.
      	   )")

      .def("get_waypoint_durations", &robot_trajectory::RobotTrajectory::getWayPointDurations,
           R"(
           Get the durations from the previous waypoint in the trajectory.

           Returns:
               list of float: The duration from previous of each waypoint in the trajectory.
           )");
  // TODO (peterdavidfagan): support other methods such as appending trajectories

  // Collision Request
  py::class_<collision_detection::CollisionRequest>(m, "CollisionRequest", R"(
      Representation of a collision checking request.
      )")

      .def(py::init<>())

      .def_readwrite("joint_model_group_name", &collision_detection::CollisionRequest::group_name,
                     R"(
                     str: The group name to check collisions for (optional; if empty, assume the complete robot)
                     )")

      .def_readwrite("distance", &collision_detection::CollisionRequest::distance,
                     R"(
                     bool: If true, compute proximity distance.
                     )")

      .def_readwrite("cost", &collision_detection::CollisionRequest::cost,
                     R"(
                     bool: If true, a collision cost is computed.
                     )")

      .def_readwrite("contacts", &collision_detection::CollisionRequest::contacts,
                     R"(
                     bool: If true, compute contacts.
                     )")

      .def_readwrite("max_contacts", &collision_detection::CollisionRequest::max_contacts,
                     R"(
                     int: Overall maximum number of contacts to compute.
                     )")

      .def_readwrite("max_contacts_per_pair", &collision_detection::CollisionRequest::max_contacts_per_pair,
                     R"(
                     int: Maximum number of contacts to compute per pair of bodies (multiple bodies may be in contact at different configurations).
                     )")

      .def_readwrite("max_cost_sources", &collision_detection::CollisionRequest::max_cost_sources,
                     R"(
                     int: When costs are computed, this value defines how many of the top cost sources should be returned.
                     )")

      // TODO (peterdavidfagan): define is_done as function call.
      //.def_readwrite("is_done", &collision_detection::CollisionRequest::is_done,
      //               R"(
      //               )")

      .def_readwrite("verbose", &collision_detection::CollisionRequest::verbose,
                     R"(
                     bool: Flag indicating whether information about detected collisions should be reported.
                     )");

  // Collision Result
  py::class_<collision_detection::CollisionResult>(m, "CollisionResult", R"(
      Representation of a collision checking result.
      )")
      .def_readwrite("collision", &collision_detection::CollisionResult::collision,
                     R"(
                     bool: True if collision was found, false otherwise.
                     )")

      .def_readwrite("distance", &collision_detection::CollisionResult::distance,
                     R"(
                     float: Closest distance between two bodies.
                     )")

      .def_readwrite("contact_count", &collision_detection::CollisionResult::contact_count,
                     R"(
                     int: Number of contacts returned.
                     )")

      // TODO (peterdavidfagan): define binding and test for ContactMap.
      .def_readwrite("contacts", &collision_detection::CollisionResult::contacts,
                     R"(
                     dict: A dict returning the pairs of ids of the bodies in contact, plus information about the contacts themselves.
                     )")

      .def_readwrite("cost_sources", &collision_detection::CollisionResult::cost_sources,
                     R"(
                     dict: The individual cost sources from computed costs.
                     )");

  // Allowed Collision Matrix
  py::class_<collision_detection::AllowedCollisionMatrix, std::shared_ptr<collision_detection::AllowedCollisionMatrix>>(
      m, "AllowedCollisionMatrix",
      R"(
          Definition of a structure for the allowed collision matrix. All elements in the collision world are referred to by their names. This class represents which collisions are allowed to happen and which are not.
          )")
      .def(py::init<std::vector<std::string>&, bool>(),
           R"(
       Initialize the allowed collision matrix using a list of names of collision objects.

       Args:
           names (list of str): A list of names of the objects in the collision world (corresponding to object IDs in the collision world).
           allowed (bool): If false, indicates that collisions between all elements must be checked for and no collisions will be ignored.
       )",
           py::arg("names"), py::arg("default_entry") = false);

  // Planning Scene
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

