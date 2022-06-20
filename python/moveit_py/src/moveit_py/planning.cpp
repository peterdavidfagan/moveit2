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
#include <utils.h>

#include "moveit_ros/moveit_cpp/planning_component.h"
#include "moveit_ros/moveit_cpp/moveit_cpp.h"
#include "moveit_ros/planning_scene_monitor/planning_scene_monitor.h"

namespace py = pybind11;

PYBIND11_MODULE(planning, m)
{
  m.doc() = "Planning components of moveit_py.";

  // Disable automatic function signatures
  // py::options options;
  // options.disable_function_signatures();

  // importing useful definitions and utilties from other modules
  auto core = py::module::import("moveit_py.core");
  auto utils = py::module::import("moveit_py.utils");

  // PlanSolution
  py::class_<moveit_cpp::PlanningComponent::PlanSolution, std::shared_ptr<moveit_cpp::PlanningComponent::PlanSolution>>(
      m, "PlanSolution")
      .def_readwrite("start_state", &moveit_cpp::PlanningComponent::PlanSolution::start_state,
                     "The starting state of the plan.")
      .def_readwrite("trajectory", &moveit_cpp::PlanningComponent::PlanSolution::trajectory,
                     "The trajectory associated to the plan solution.")
      .def_readwrite("error_code", &moveit_cpp::PlanningComponent::PlanSolution::error_code,
                     "The error code associate to the plan solution.")
      .def(
          "__bool__",
          [](moveit_cpp::PlanningComponent::PlanSolution& plan_solution) { return bool(plan_solution.error_code); },
          py::is_operator(), "Returns True when the plan is successful.");

  // CurrentStateMonitor
  // py::class_<planning_scene_monitor::CurrentStateMonitor,
  // std::shared_ptr<planning_scene_monitor::CurrentStateMonitor>>(m, "CurrentStateMonitor")

  // PlanningSceneMonitor
  py::class_<planning_scene_monitor::PlanningSceneMonitor, std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>>(
      m, "PlanningSceneMonitor")
      //.def(py::init<>()) for now this object is only accessible through the moveit_py.planning.MoveItPy class
      .def_property("name", &planning_scene_monitor::PlanningSceneMonitor::getName, nullptr,
                    "The name of the planning scene monitor.");
  //.def("get_state_monitor", &planning_scene_monitor::PlanningSceneMonitor::getStateMonitor,
  //   py::return_value_policy::reference_internal,
  //	   "Returns the state monitor of the planning scene monitor.")
  //.def("update_frame_transforms")
  //.def("clearOctomap")
  //

  // PlanningComponent
  py::class_<moveit_cpp::PlanningComponent, std::shared_ptr<moveit_cpp::PlanningComponent>>(m, "PlanningComponent")
      .def(py::init<const std::string&, const std::shared_ptr<moveit_cpp::MoveItCpp>&>(), py::arg("group_name"),
           py::arg("moveit_cpp"), "Constructs a PlanningComponent instance.")

      // start state methods
      .def("set_start_state_to_current_state", &moveit_cpp::PlanningComponent::setStartStateToCurrentState,
           "Set the start state of the plan to the current state from the PlanningSceneMonitor. Does so by resetting "
           "the consider_start_state_ pointer for the planning component instance.")

      .def("set_start_state", py::overload_cast<const std::string&>(&moveit_cpp::PlanningComponent::setStartState),
           py::arg("start_state_name"), py::return_value_policy::move,
           "Set the start state to a predefined state from the robot srdf.")

      .def("set_start_state",
           py::overload_cast<const moveit::core::RobotState&>(&moveit_cpp::PlanningComponent::setStartState),
           py::arg("start_state"), py::return_value_policy::move, "Set the start state to a given RobotState.")

      .def("get_start_state", &moveit_cpp::PlanningComponent::getStartState,
           py::return_value_policy::reference_internal, "Returns the current start state of the PlanningComponent.")

      // goal state methods
      .def("set_goal", py::overload_cast<const std::string&>(&moveit_cpp::PlanningComponent::setGoal),
           py::arg("goal_state_name"), py::return_value_policy::move,
           "Set the goal state using predefined state from robot srdf.")
      .def("set_goal", py::overload_cast<const moveit::core::RobotState&>(&moveit_cpp::PlanningComponent::setGoal),
           py::arg("goal_state"), py::return_value_policy::move, "Set the goal state to a given RobotState.")
      .def("set_goal",
           py::overload_cast<std::shared_ptr<moveit_cpp::PlanningComponent>&, py::object&, std::string>(&set_goal),
           py::arg("goal_pose"), py::arg("link_name"), py::return_value_policy::move,
           "Set the goal pose of the plan by specifying the goal pose of a given link.")
      .def("set_goal", py::overload_cast<std::shared_ptr<moveit_cpp::PlanningComponent>&, py::list&>(&set_goal),
           py::arg("goal_constraints"), py::return_value_policy::move,
           "Set the goal pose of the plan by specifying the kinematic constraints.")

      // plan/execution methods
      .def("plan", py::overload_cast<>(&moveit_cpp::PlanningComponent::plan), py::return_value_policy::move,
           "Attempt to generate motion plan solution.")
      .def("execute", &moveit_cpp::PlanningComponent::execute, py::arg("blocking") = true,
           py::return_value_policy::move, "Execute successfully planned motion plan.")

      // retrieving meta data
      .def("get_named_target_state", &moveit_cpp::PlanningComponent::getNamedTargetStates,
           py::return_value_policy::move, "Return the named target state.")
      .def("get_named_target_state_values", &moveit_cpp::PlanningComponent::getNamedTargetStateValues,
           py::return_value_policy::move, "Return values of named target state.")
      .def("get_planning_group_name", &moveit_cpp::PlanningComponent::getPlanningGroupName,
           "Return planning group name.")

      // Interacting with workspace
      .def("set_workspace", &moveit_cpp::PlanningComponent::setWorkspace, py::arg("min_x"), py::arg("min_y"),
           py::arg("min_z"), py::arg("max_x"), py::arg("max_y"), py::arg("max_z"), "Sets the workspace parameters.")
      .def("unset_workspace", &moveit_cpp::PlanningComponent::unsetWorkspace, "Unsets the workspace parameters");

  // MoveItPy
  py::class_<moveit_cpp::MoveItCpp, std::shared_ptr<moveit_cpp::MoveItCpp>>(m, "MoveItPy")
      .def(py::init([](std::string node_name, std::string launch_params_filepath, bool provide_planning_service) {
             static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_initializer");

             RCLCPP_INFO(LOGGER, "Initialize rclcpp");
             rclcpp::init(0, nullptr);

             RCLCPP_INFO(LOGGER, "Initialize node parameters");
             rclcpp::NodeOptions node_options;

             if (!launch_params_filepath.empty())
             {
               node_options.allow_undeclared_parameters(true)
                   .automatically_declare_parameters_from_overrides(true)
                   .arguments({ "--ros-args", "--params-file", launch_params_filepath });
             }
             else
             {
               node_options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
             }
             RCLCPP_INFO(LOGGER, "Initialize node");
             rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(node_name, "", node_options);

             RCLCPP_INFO(LOGGER, "Spin separate thread");
             auto spin_node = [&node]() {
               rclcpp::executors::SingleThreadedExecutor executor;
               executor.add_node(node);
               executor.spin();
             };
             std::thread execution_thread(spin_node);
             execution_thread.detach();

             std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);

             if (provide_planning_service)
             {
               moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();
             };

             return moveit_cpp_ptr;
           }),
           py::arg("node_name") = "moveit_py",
           py::arg("launch_params_filepath") = utils.attr("get_launch_params_filepath")().cast<std::string>(),
           py::arg("provide_planning_service") = true, py::return_value_policy::reference,
           "Initialize moveitcpp and planning scene service. Returns a Python instance that binds the methods "
           "available in moveit_cpp.")

      .def("get_planning_component", &get_planning_component, py::return_value_policy::take_ownership,
           "Returns PlanningComponent instance.")
      .def("get_planning_pipeline_names", &moveit_cpp::MoveItCpp::getPlanningPipelineNames,
           py::return_value_policy::move, "Return list of planning pipeline names")
      .def(
          "shutdown", [](std::shared_ptr<moveit_cpp::MoveItCpp>& moveit_cpp) { rclcpp::shutdown(); },
          "Shutdown all spinning process on the C++ side.")
      .def("get_current_state",
           py::overload_cast<moveit::core::RobotStatePtr&, double>(&moveit_cpp::MoveItCpp::getCurrentState),
           py::arg("current_state"), py::arg("wait_seconds") = 0.0, py::return_value_policy::take_ownership,
           "Returns the current state from planning scene monitor.")
      .def("get_current_state", py::overload_cast<double>(&moveit_cpp::MoveItCpp::getCurrentState),
           py::arg("wait_seconds") = 0.0, py::return_value_policy::take_ownership)
      .def("get_planning_scene_monitor", &moveit_cpp::MoveItCpp::getPlanningSceneMonitor,
           py::return_value_policy::reference)
      //.def("get_trajectory_execution_manager",
      //&moveit_cpp::MoveItCpp::getTrajectoryExecutionManager, py::return_value_policy::reference)
      .def("get_robot_model", &moveit_cpp::MoveItCpp::getRobotModel, py::return_value_policy::reference,
           "Returns robot model.");
}
