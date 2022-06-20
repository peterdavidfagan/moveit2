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
#include <moveit/planning_scene/planning_scene.h>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <rclcpp/rclcpp.hpp>
#include <serialize_ros_msg.h>

#include "moveit_ros/moveit_cpp/moveit_cpp.h"
#include "moveit_ros/moveit_cpp/planning_component.h"
#include "moveit_ros/planning_scene_monitor/planning_scene_monitor.h"

PYBIND11_MODULE(planning, m)
{
  m.doc() = "Python bindings for moveit_cpp functionalities.";

  // Provide custom function signatures
  py::options options;
  options.disable_function_signatures();

  // importing useful python modules
  auto core = py::module::import("moveit_py.core");
  auto utils = py::module::import("moveit_py.utils");

  // PlanRequestParameters (no property definitions as implicit in parameter name)
  py::class_<moveit_cpp::PlanningComponent::PlanRequestParameters,
             std::shared_ptr<moveit_cpp::PlanningComponent::PlanRequestParameters>>(m, "PlanRequestParameters",
                                                                                    R"(
			     Planner parameters provided with a MotionPlanRequest.
			     )")

      .def(py::init<>())
      .def_readwrite("planner_id", &moveit_cpp::PlanningComponent::PlanRequestParameters::planner_id,
                     R"(
                     str: The planner id to use.
                     )")
      .def_readwrite("planning_pipeline", &moveit_cpp::PlanningComponent::PlanRequestParameters::planning_pipeline,
                     R"(
                     str: The planning pipeline to use.
                     )")
      .def_readwrite("planning_attempts", &moveit_cpp::PlanningComponent::PlanRequestParameters::planning_attempts,
                     R"(
                     int: The number of planning attempts to make.
                     )")
      .def_readwrite("planning_time", &moveit_cpp::PlanningComponent::PlanRequestParameters::planning_time,
                     R"(
                     float: The amount of time to spend planning.
                     )")
      .def_readwrite("max_velocity_scaling_factor",
                     &moveit_cpp::PlanningComponent::PlanRequestParameters::max_velocity_scaling_factor,
                     R"(
                     float: The maximum velocity scaling factor that can be used.
                     )")
      .def_readwrite("max_acceleration_scaling_factor",
                     &moveit_cpp::PlanningComponent::PlanRequestParameters::max_acceleration_scaling_factor,
                     R"(
                     float: The maximum scaling factor that can be used.
                     )");

  // PlanSolution
  py::class_<moveit_cpp::PlanningComponent::PlanSolution, std::shared_ptr<moveit_cpp::PlanningComponent::PlanSolution>>(
      m, "PlanSolution", R"( 
      Representation of a plan solution.
      )")

      .def(py::init<>())
      .def_property("start_state", &moveit_py::bind_moveit_cpp::get_plan_solution_start_state, nullptr,
                    py::return_value_policy::copy,
                    R"(
                    moveit_msgs.msg.RobotState: The start state of the plan.
                    )")
      .def_property("trajectory", &moveit_py::bind_moveit_cpp::get_plan_solution_trajectory, nullptr,
                    py::return_value_policy::copy,
                    R"(
                    :py:class:`moveit_py.core.RobotTrajectory`: The trajectory of the plan.
                    )")

      // TODO (peterdavidfagan): Add bindings for error codes.
      .def_property("error_code", &moveit_py::bind_moveit_cpp::get_plan_solution_error_code, nullptr,
                    py::return_value_policy::copy,
                    R"(
                    moveit_py.core.ErrorCode: The reason why the plan failed.
                    )")
      .def(
          "__bool__",
          [](moveit_cpp::PlanningComponent::PlanSolution& plan_solution) { return bool(plan_solution.error_code); },
          py::is_operator(),
          R"(
          Returns:
              bool: True if the plan succeeded otherwise false.
          )");

  // In Python we lock the planning scene using a with statement as this allows us to have control over resources.
  // To this end each of the below manager classes binds special methods __enter__ and __exit__.
  // LockedPlanningSceneContextManagerRO
  py::class_<moveit_py::bind_planning_scene_monitor::LockedPlanningSceneContextManagerRO>(
      m, "LockedPlanningSceneContextManagerRO", R"(
      A context manager that locks the planning scene for reading.
      )")

      .def("__enter__",
           &moveit_py::bind_planning_scene_monitor::LockedPlanningSceneContextManagerRO::locked_planning_scene_ro_enter_,
           R"(
           Special method that is used with the with statement, provides access to a locked plannning scene instance.

           Returns:
               :py:class:`moveit_py.core.PlanningScene`: The locked planning scene.
        )")
      .def("__exit__",
           &moveit_py::bind_planning_scene_monitor::LockedPlanningSceneContextManagerRO::locked_planning_scene_ro_exit_,
           R"(
           Special method that is used with the with statement, releases the lock on the planning scene.
           )");

  // LockedPlanningSceneContextManagerRW
  py::class_<moveit_py::bind_planning_scene_monitor::LockedPlanningSceneContextManagerRW>(
      m, "LockedPlanningSceneContextManagerRW", R"(
      A context manager that locks the planning scene for reading and writing.
      )")

      .def("__enter__",
           &moveit_py::bind_planning_scene_monitor::LockedPlanningSceneContextManagerRW::locked_planning_scene_rw_enter_,
           R"(
           Special method that is used with the with statement, provides access to a locked plannning scene instance.

           Returns:
               :py:class:`moveit_py.core.PlanningScene`: The locked planning scene.
           )")

      .def("__exit__",
           &moveit_py::bind_planning_scene_monitor::LockedPlanningSceneContextManagerRW::locked_planning_scene_rw_exit_,
           R"(
           Special method that is used with the with statement, releases the lock on the planning scene.
           )");

  // PlanningSceneMonitor
  py::class_<planning_scene_monitor::PlanningSceneMonitor, planning_scene_monitor::PlanningSceneMonitorPtr>(
      m, "PlanningSceneMonitor", R"(
      Maintains the internal state of the planning scene.
      )")

      .def_property("name", &planning_scene_monitor::PlanningSceneMonitor::getName, nullptr,
                    R"(
                    str: The name of this planning scene monitor.
                    )")

      .def("start_scene_monitor", &planning_scene_monitor::PlanningSceneMonitor::startSceneMonitor,
           R"(
           Starts the scene monitor.
           )")

      .def("stop_scene_monitor", &planning_scene_monitor::PlanningSceneMonitor::stopSceneMonitor,
           R"(
           Stops the scene monitor.
           )")

      .def("start_state_monitor", &planning_scene_monitor::PlanningSceneMonitor::startStateMonitor,
           R"(
	   Starts the state monitor.
	   )")

      .def("stop_state_monitor", &planning_scene_monitor::PlanningSceneMonitor::stopStateMonitor,
           R"(
	       Stops the state monitor.
	   )")

      .def("wait_for_current_robot_state", &planning_scene_monitor::PlanningSceneMonitor::waitForCurrentRobotState,
           R"(
	   Waits for the current robot state to be received.
	   )")

      .def("clear_octomap", &planning_scene_monitor::PlanningSceneMonitor::clearOctomap,
           R"(
           Clears the octomap.
           )")

      .def("read_only", &moveit_py::bind_planning_scene_monitor::read_only,
           R"(
           Returns a read-only context manager for the planning scene.
           )")

      .def("read_write", &moveit_py::bind_planning_scene_monitor::read_write,
           R"(
           Returns a read-write context manager for the planning scene.
           )");

  // PlanningComponent
  py::class_<moveit_cpp::PlanningComponent, std::shared_ptr<moveit_cpp::PlanningComponent>>(m, "PlanningComponent",
                                                                                            R"(
      Represents a joint model group and motion plans corresponding to this joint model group.
      )")

      .def(py::init<const std::string&, const std::shared_ptr<moveit_cpp::MoveItCpp>&>(),
           py::arg("joint_model_group_name"), py::arg("moveit_py_instance"),
           R"(
            Constructs a PlanningComponent instance.

            Args:
                joint_model_group_name (str): The name of the joint model group to plan for.
                moveit_py_instance (:py:class:`moveit_py.core.MoveItPy`): The MoveItPy instance to use.
        )")

      .def_property("planning_group_name", &moveit_cpp::PlanningComponent::getPlanningGroupName, nullptr,
                    R"(
                    str: The name of the planning group to plan for.
                    )")

      .def_property("named_target_states", &moveit_cpp::PlanningComponent::getNamedTargetStates, nullptr,
                    R"(
                    list of str: The names of the named robot states available as targets.
                    )")

      // TODO (peterdavidfagan): write test case for this method.
      .def_property("named_target_state_values", &moveit_cpp::PlanningComponent::getNamedTargetStateValues, nullptr,
                    py::return_value_policy::move,
                    R"(
                    dict: The joint values for targets specified by name.
                    )")

      // start state methods
      .def("set_start_state_to_current_state", &moveit_cpp::PlanningComponent::setStartStateToCurrentState,
           R"(
           Set the start state of the plan to the current state of the robot. 
           )")

      .def("set_start_state", py::overload_cast<const std::string&>(&moveit_cpp::PlanningComponent::setStartState),
           py::arg("start_state_name"), py::return_value_policy::move,
           R"(
           Set the start state to a predefined state from the robot srdf.

           Args:
               start_state_name (str):
           )")

      .def("set_start_state",
           py::overload_cast<const moveit::core::RobotState&>(&moveit_cpp::PlanningComponent::setStartState),
           py::arg("start_state"), py::return_value_policy::move,
           R"(
           Set the start state to a given RobotState.

           Args:
               start_state (moveit_py.core.RobotState): The state to set the start state to.
           )")

      .def("get_start_state", &moveit_cpp::PlanningComponent::getStartState,
           py::return_value_policy::reference_internal,
           R"(
           Returns the current start state for the planning component.
           )")

      // goal state methods
      .def("set_goal", py::overload_cast<const std::string&>(&moveit_cpp::PlanningComponent::setGoal),
           py::arg("goal_state_name"), py::return_value_policy::move,
           R"(
           Set the goal state using predefined state from robot srdf.

           Args:
               goal_state_name (str): The name of the goal state.
           )")

      .def("set_goal", py::overload_cast<const moveit::core::RobotState&>(&moveit_cpp::PlanningComponent::setGoal),
           py::arg("goal_state"), py::return_value_policy::move,
           R"(
           Set the goal state to a given RobotState.

           Args:
               goal_state (moveit_py.core.RobotState): The state to set the goal state to.
           )")

      .def("set_goal",
           py::overload_cast<std::shared_ptr<moveit_cpp::PlanningComponent>&, py::object&, std::string>(
               &moveit_py::bind_planning_component::set_goal),
           py::arg("goal_pose_msg"), py::arg("link_name"), py::return_value_policy::move,
           R"(
           Set the goal pose of the plan by specifying the goal pose of a given link.

           Args:
               goal_pose_msg (geometry_msgs.msg.Pose): The goal pose.
               link_name (str): The name of the link to set the goal pose of.
        )")

      .def("set_goal",
           py::overload_cast<std::shared_ptr<moveit_cpp::PlanningComponent>&, py::list&>(
               &moveit_py::bind_planning_component::set_goal),
           py::arg("goal_constraints"), py::return_value_policy::move,
           R"(
           Set the goal pose of the plan by specifying the kinematic constraints.

           Args:
                goal_constraints (list of moveit_msgs.msg.Constraints): The goal constraints.
        )")

      .def("set_goal",
           py::overload_cast<std::shared_ptr<moveit_cpp::PlanningComponent>&, py::array_t<double>, std::string&>(
               &moveit_py::bind_planning_component::set_goal),
           py::arg("goal_pose"), py::arg("link_name"),
           R"(
           Set the goal pose of the plan through providing a number array containing x, y, z, w pose variables in that order along with the associated link.

           Args:
               goal_pose (:py:class:`numpy.ndarray`): The goal pose.
               link_name (str): The name of the link to set the goal pose of.
        )")

      // plan/execution methods
      .def("plan", py::overload_cast<>(&moveit_cpp::PlanningComponent::plan), py::return_value_policy::move,
           R"(
           Run a plan from start or current state to fulfill the last goal constraints provided by the set_goal method using default parameters.

           Returns:
               :py:class:`moveit_py.planning.PlanSolution`: The plan solution.
        )")

      .def("plan",
           py::overload_cast<const moveit_cpp::PlanningComponent::PlanRequestParameters&>(
               &moveit_cpp::PlanningComponent::plan),
           py::return_value_policy::move,
           R"(
           Run a plan from start or current state to fulfill the last goal constraints provided by the set_goal method using the provided :py:class:`moveit_py.planning.PlanRequestParameters`.

           Args:
               parameters (:py:class:`moveit_py.planning.PlanRequestParameters`): The parameters to use for the plan.
        )")

      .def("set_path_constraints", &moveit_py::bind_planning_component::set_path_constraints,
           py::arg("path_constraints"), py::return_value_policy::move,
           R"(
           Set the path constraints generated from a moveit msg Constraints.

           Args:
               path_constraints (moveit_msgs.msg.Constraints): The path constraints.
        )")

      .def("execute", &moveit_cpp::PlanningComponent::execute, py::arg("blocking") = true,
           py::return_value_policy::move,
           R"(
           Execute the latest computed solution trajectory computed by plan(). 

           By default this function terminates after the execution is complete. The execution can be run in background by setting blocking to false.

           Args:
               blocking (bool): Whether to wait for the execution to complete or not.
        )")

      // Interacting with workspace
      .def("set_workspace", &moveit_cpp::PlanningComponent::setWorkspace, py::arg("min_x"), py::arg("min_y"),
           py::arg("min_z"), py::arg("max_x"), py::arg("max_y"), py::arg("max_z"),
           R"(
           Specify the workspace bounding box. 

           The box is specified in the planning frame (i.e. relative to the robot root link start position). The workspace applies only to the root joint of a mobile robot (driving base, quadrotor) and does not limit the workspace of a robot arm.

           Args:
               min_x (float): The minimum x value of the workspace.
               min_y (float): The minimum y value of the workspace.
               min_z (float): The minimum z value of the workspace.
               max_x (float): The maximum x value of the workspace.
               max_y (float): The maximum y value of the workspace.
               max_z (float): The maximum z value of the workspace.
        )")

      .def("unset_workspace", &moveit_cpp::PlanningComponent::unsetWorkspace,
           R"(
           Remove the workspace bounding box from planning.
           )");

  // MoveItPy
  py::class_<moveit_cpp::MoveItCpp, std::shared_ptr<moveit_cpp::MoveItCpp>>(m, "MoveItPy", R"(
  The MoveItPy class is the main interface to the MoveIt Python API. It is a wrapper around the MoveIt C++ API.
									     )")

      .def(py::init([](std::string node_name, std::string launch_params_filepath, py::object config_dict,
                       bool provide_planning_service) {
             static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_initializer");

             RCLCPP_INFO(LOGGER, "Initialize rclcpp");
             rclcpp::init(0, nullptr);

             RCLCPP_INFO(LOGGER, "Initialize node parameters");
             rclcpp::NodeOptions node_options;

             // This section is used to load the appropriate node parameters before spinning a moveit_cpp instance
             // Priority is given to parameters supplied directly via a config_dict, followed by launch parameters
             // and finally no supplied parameters.
             if (!config_dict.is(py::none()))
             {
               // TODO (peterdavidfagan): replace python method with C++ method
               auto utils = py::module::import("moveit_py.utils");
               std::string params_filepath =
                   utils.attr("create_params_file_from_dict")(config_dict, "moveit_py").cast<std::string>();
               RCLCPP_INFO(LOGGER, "params_filepath: %s", params_filepath.c_str());
               node_options.allow_undeclared_parameters(true)
                   .automatically_declare_parameters_from_overrides(true)
                   .arguments({ "--ros-args", "--params-file", params_filepath });
             }
             else if (!launch_params_filepath.empty())
             {
               node_options.allow_undeclared_parameters(true)
                   .automatically_declare_parameters_from_overrides(true)
                   .arguments({ "--ros-args", "--params-file", launch_params_filepath });
             }
             else
             {
               // TODO (peterdavidfagan): consider failing initialization if no params file is provided
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
           py::arg("config_dict") = py::none(), py::arg("provide_planning_service") = true,
           py::return_value_policy::reference,
           R"(
           Initialize moveit_cpp node and the planning scene service. 
           )")

      .def("get_planning_component", &moveit_py::bind_moveit_cpp::get_planning_component,
           py::arg("planning_component_name"), py::return_value_policy::take_ownership,
           R"(
           Creates a planning component instance.

           Args:
               planning_component_name (str): The name of the planning component.

           Returns:
               :py:class:`moveit_py.planning.PlanningComponent`: A planning component instance corresponding to the provided plan component name.
          )")

      .def("get_planning_pipeline_names", &moveit_cpp::MoveItCpp::getPlanningPipelineNames,
           py::arg("joint_model_group_name"), py::return_value_policy::move,
           R"(
           Return list of planning pipeline names for the provided joint model group name.

           Args:
               joint_model_group_name (str): The name of the joint model group.

           Returns:
               list of str: List of planning pipeline names.
        )")

      .def(
          "shutdown", [](std::shared_ptr<moveit_cpp::MoveItCpp>& moveit_cpp) { rclcpp::shutdown(); },
          R"(
          Shutdown the moveit_cpp node.
          )")

      // TODO (peterdavidfagan): recreate as properties
      //.def("get_current_state",
      //     py::overload_cast<moveit::core::RobotStatePtr&, double>(&moveit_cpp::MoveItCpp::getCurrentState),
      //     py::arg("current_state"), py::arg("wait_seconds") = 0.0, py::return_value_policy::take_ownership,
      //     R"(
      //         Returns the current state from planning scene monitor.
      //     )")

      //.def("get_current_state", py::overload_cast<double>(&moveit_cpp::MoveItCpp::getCurrentState),
      //     py::arg("wait_seconds") = 0.0, py::return_value_policy::take_ownership,
      //     R"(
      //     )")

      .def("get_planning_scene_monitor", &moveit_cpp::MoveItCpp::getPlanningSceneMonitor,
           py::return_value_policy::reference,
           R"(
           Returns the planning scene monitor.
           )")

      .def("get_robot_model", &moveit_cpp::MoveItCpp::getRobotModel, py::return_value_policy::reference,
           R"(
           Returns robot model.
        )");
}
