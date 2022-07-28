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
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_state/robot_state.h>
#include <copy_ros_msg.h>
#include <serialize_ros_msg.h>

#include "moveit_core/robot_state/robot_state.h"
#include "moveit_core/robot_model/robot_model.h"

PYBIND11_MODULE(core, m)
{
  // TODO(Peter): convert all get/set methods to properties
  m.doc() = "Python bindings for moveit_core functionalities.";

  // Disable automatic function signatures
  // py::options options;
  // options.disable_function_signatures();

  // LinkModel
  // py::class_<moveit::core::LinkModel>(m, "LinkModel").def("get_name", moveit::core::LinkModel::getName);

  // JointModel (this is an abstract base class)
  // TODO(Peter): complete set of methods
  // py::class_<moveit::core::JointModel>(m, "JointModel");

  // JointModelGroup
  py::class_<moveit::core::JointModelGroup>(m, "JointModelGroup")
      .def("get_name", &moveit::core::JointModelGroup::getName)
      .def("get_joint_model_names", &moveit::core::JointModelGroup::getJointModelNames);

  // RobotModel
  py::class_<moveit::core::RobotModel, std::shared_ptr<moveit::core::RobotModel>>(m, "RobotModel")
      .def(py::init<const urdf::ModelInterfaceSharedPtr&, const srdf::ModelConstSharedPtr&>(), py::arg("urdf_model"),
           py::arg("srdf_model"))
      .def("get_name", &moveit::core::RobotModel::getName, "Returns the name of the robot.")
      .def("get_model_frame", &moveit::core::RobotModel::getModelFrame,
           "Returns the frame in which the transforms for this robot model are computed.")
      .def(
          "printModelInfo",
          [](std::shared_ptr<moveit::core::RobotModel>& s) {
            std::stringstream ss;
            s->printModelInfo(ss);
            return ss.str();
          },
          py::return_value_policy::move, "Returns a string containing robot model information.")
      .def("get_urdf", &moveit::core::RobotModel::getURDF, "Return the parsed URDF model.")
      .def("get_srdf", &moveit::core::RobotModel::getSRDF, "Return the parsed SRDF model.")
      .def("get_root_joint", &moveit::core::RobotModel::getRootJoint, "Return the root joint.")
      .def("get_root_joint_name", &moveit::core::RobotModel::getRootJointName, "Return the root joint name.")

      // Interacting with joint model groups
      .def("has_joint_model_group", &moveit::core::RobotModel::hasJointModelGroup, py::arg("joint_model_group_name"),
           "Returns true if joint model group exists.")
      .def("get_joint_model_group_names", &moveit::core::RobotModel::getJointModelGroupNames,
           "Return a list of joint names in the order they appear in the robot state.")
      .def("get_joint_model_group",
           py::overload_cast<const std::string&>(&moveit::core::RobotModel::getJointModelGroup),
           py::arg("joint_model_group_name"), py::return_value_policy::reference_internal,
           "Returns joint model group by name from robot model.")
      .def("get_joint_model_groups", py::overload_cast<>(&moveit::core::RobotModel::getJointModelGroups),
           py::return_value_policy::move, "Returns a list of joint model groups.");

  // RobotState
  py::class_<moveit::core::RobotState, std::shared_ptr<moveit::core::RobotState>>(m, "RobotState")
      .def(py::init<const std::shared_ptr<const moveit::core::RobotModel>&>())

      // Get underlying robot model, frame transformations and jacobian
      .def("get_robot_model", &moveit::core::RobotState::getRobotModel, py::return_value_policy::reference,
           "Get the robot model this state is contructed for.")
      .def("get_frame_transform", &get_frame_transform, py::arg("frame_id"), py::return_value_policy::move,
           "Get the transformation matrix from the model frame to the frame identified by frame_id.")
      .def("get_jacobian",
           py::overload_cast<std::shared_ptr<moveit::core::RobotState>&, const std::string&, const Eigen::Vector3d&>(
               &get_jacobian),
           py::arg("joint_group_name"), py::arg("reference_point_position"), py::return_value_policy::move,
           "Compute the Jacobian with reference to the last link of a specified group. If the group is not a chain, an "
           "exception is thrown.")
      .def("get_jacobian",
           py::overload_cast<std::shared_ptr<moveit::core::RobotState>&, const std::string&, const std::string,
                             const Eigen::Vector3d&, bool>(&get_jacobian),
           py::arg("joint_model_group_name"), py::arg("link_name"), py::arg("reference_point_position"),
           py::arg("use_quaternion_representation") = false, py::return_value_policy::move,
           "Compute the Jacobian with reference to a particular point on a given link, for a specified group.")

      // Get state information
      .def("get_state_tree", py::overload_cast<>(&moveit::core::RobotState::getStateTreeString, py::const_),
           py::return_value_policy::move, "Returns the state tree string.")
      .def(
          "print_state_info",
          [](std::shared_ptr<moveit::core::RobotState>& s) {
            std::stringstream ss;
            s->printStateInfo(ss);
            return ss.str();
          },
          py::return_value_policy::move, "Returns a string containing the current state information.")

      // Getting and setting joint model group positions, velocities, accelerations
      .def("set_joint_group_positions",
           py::overload_cast<const std::string&, const Eigen::VectorXd&>(
               &moveit::core::RobotState::setJointGroupPositions),
           py::arg("joint_group_name"), py::arg("position_values"),
           "Given positions for the variables of active joints that make up a group, in the order found in the group "
           "(excluding values of mimic joints), set those as the new values that correspond to the group.")
      .def("set_joint_group_active_positions",
           py::overload_cast<const std::string&, const Eigen::VectorXd&>(
               &moveit::core::RobotState::setJointGroupActivePositions),
           py::arg("joint_group_name"), py::arg("position_values"),
           "Given positions for the variables of active joints that make up a group, in the order found in the group "
           "(excluding values of mimic joints), set those as the new values that correspond to the group.")
      .def("copy_joint_group_positions", &copy_joint_group_positions, py::arg("joint_group_name"),
           "For a given group, copy the position values of the variables that make up the group")
      .def("set_joint_group_velocities",
           py::overload_cast<const std::string&, const Eigen::VectorXd&>(
               &moveit::core::RobotState::setJointGroupVelocities),
           py::arg("joint_group_name"), py::arg("velocity_values"),
           "Given velocities for the variables of active joints that make up a group, in the order found in the group "
           "(excluding values of mimic joints), set those as the new values that correspond to the group.")
      .def("copy_joint_group_velocities", &copy_joint_group_velocities, py::arg("joint_group_name"),
           "For a given group, copy the velocity values of the variables that make up the group")
      .def("set_joint_group_accelerations",
           py::overload_cast<const std::string&, const Eigen::VectorXd&>(
               &moveit::core::RobotState::setJointGroupAccelerations),
           py::arg("joint_group_name"), py::arg("acceleration_values"),
           "Given accelerations for the variables of active joints that make up a group, in the order found in the "
           "group (excluding values of mimic joints), set those as the new values that correspond to the group.")
      .def("copy_joint_group_accelerations", &copy_joint_group_accelerations, py::arg("joint_group_name"),
           "For a given group, copy the acceleration values of the variables that make up the group")

      // Setting state from inverse kinematics
      //.def("set_from_ik",
      //     py::overload_cast<std::shared_ptr<moveit::core::RobotState>&, const std::string&, py::object&,
      //                       const std::string&, const double, const moveit::core::GroupStateValidityCallbackFn&,
      //                       const kinematics::KinematicsQueryOptions&>(&set_from_ik),
      //     py::arg("joint_model_group_name"), py::arg("geometry_pose"), py::arg("tip_name"), py::arg("timeout") = 0.0,
      //     py::arg("constraint") = moveit::core::GroupStateValidityCallbackFn(),
      //     py::arg("options") = kinematics::KinematicsQueryOptions(),
      //     "If the group this state corresponds to is a chain and a solver is available, then the joint values can be
      //     " "set by computing inverse kinematics. The pose is assumed to be in the reference frame of the kinematic "
      //     "model. Returns true on success.")

      // Setting entire state values
      .def("set_to_default_values", py::overload_cast<>(&moveit::core::RobotState::setToDefaultValues),
           "Sets all joints to their default positions. The default position is 0, or if this is not within the bounds "
           "then half way between the min and max bound.")
      .def("set_to_default_values",
           py::overload_cast<const moveit::core::JointModelGroup*, const std::string&>(
               &moveit::core::RobotState::setToDefaultValues),
           py::arg("joint_model_group"), py::arg("name"),
           "Set the joints in the input joint model group to their default values as defined in the srdf.")
      .def("set_to_default_values",
           py::overload_cast<std::shared_ptr<moveit::core::RobotState>&, const std::string&, const std::string&>(
               &set_to_default_values),
           py::arg("joint_model_group_name"), py::arg("name"),
           "Set the joints in the input joint model group to their default values as defined in the srdf.")
      .def("set_to_random_positions", py::overload_cast<>(&moveit::core::RobotState::setToRandomPositions),
           "Set all joints to random positions within the default bounds.")
      .def("set_to_random_positions",
           py::overload_cast<const moveit::core::JointModelGroup*>(&moveit::core::RobotState::setToRandomPositions),
           py::arg("joint_model_group"),
           "Set all joints in the joint model group to random positions within the default bounds.");
}

