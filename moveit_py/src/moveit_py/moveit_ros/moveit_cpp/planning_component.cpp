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

#include "planning_component.h"

namespace moveit_py
{
namespace bind_planning_component
{
bool set_goal(std::shared_ptr<moveit_cpp::PlanningComponent>& planning_component, py::object& pose_stamped,
              std::string link_name)
{
  // convert to C++ PoseStamped object
  geometry_msgs::msg::PoseStamped pose_stamped_cpp = PoseStampedToCpp(pose_stamped);

  // set the goal using planning component
  return planning_component->setGoal(pose_stamped_cpp, link_name);
}

bool set_goal(std::shared_ptr<moveit_cpp::PlanningComponent>& planning_component, py::list& constraints_list)
{
  // iterate through list of constraints and convert to C++
  std::vector<moveit_msgs::msg::Constraints> constraints_vec_cpp;
  for (int i = 0; i < py::len(constraints_list); i++)
  {
    moveit_msgs::msg::Constraints constraints_cpp = ConstraintsToCpp(constraints_list[i]);
    constraints_vec_cpp.push_back(constraints_cpp);
  }

  // set the goal using planning component
  return planning_component->setGoal(constraints_vec_cpp);
}

bool set_goal(std::shared_ptr<moveit_cpp::PlanningComponent>& planning_component, py::array_t<double> pose_goal,
              std::string& link_name)
{
  py::buffer_info buf = pose_goal.request();
  double* ptr = (double*)buf.ptr;

  geometry_msgs::msg::PoseStamped pose_goal_cpp;
  pose_goal_cpp.header.frame_id = link_name;
  pose_goal_cpp.pose.position.x = ptr[0];
  pose_goal_cpp.pose.position.y = ptr[1];
  pose_goal_cpp.pose.position.z = ptr[2];
  pose_goal_cpp.pose.orientation.w = ptr[3];
  return planning_component->setGoal(pose_goal_cpp, link_name);
}

bool set_path_constraints(std::shared_ptr<moveit_cpp::PlanningComponent>& planning_component,
                          py::object path_constraints)
{
  moveit_msgs::msg::Constraints path_constraints_cpp;
  py::module_ rclpy_serialization = py::module::import("rclpy.serialization");
  py::bytes serialized_msg = rclpy_serialization.attr("serialize_message")(path_constraints);
  deserializeMsg(serialized_msg, path_constraints_cpp);

  return planning_component->setPathConstraints(path_constraints_cpp);
}

void init_plan_request_parameters(py::module& m)
{
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
}

void init_plan_solution(py::module& m)
{
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
}

void init_planning_component_context_manager(py::module& m)
{
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
}

void init_planning_component(py::module& m)
{
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
}
}  // namespace bind_planning_component
}  // namespace moveit_py
