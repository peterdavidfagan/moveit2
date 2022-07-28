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

#include <utils.h>
#include "robot_state.h"

Eigen::MatrixXd get_frame_transform(std::shared_ptr<moveit::core::RobotState>& robot_state, std::string& frame_id)
{
  bool* frame_found;
  auto transformation = robot_state->getFrameTransform(frame_id, frame_found);
  return transformation.matrix();
}

Eigen::VectorXd copy_joint_group_positions(std::shared_ptr<moveit::core::RobotState>& robot_state,
                                           const std::string& joint_model_group_name)
{
  Eigen::VectorXd values;
  robot_state->copyJointGroupPositions(joint_model_group_name, values);
  return values;
}

Eigen::VectorXd copy_joint_group_velocities(std::shared_ptr<moveit::core::RobotState>& robot_state,
                                            const std::string& joint_model_group_name)
{
  Eigen::VectorXd values;
  robot_state->copyJointGroupVelocities(joint_model_group_name, values);
  return values;
}

Eigen::VectorXd copy_joint_group_accelerations(std::shared_ptr<moveit::core::RobotState>& robot_state,
                                               const std::string& joint_model_group_name)
{
  Eigen::VectorXd values;
  robot_state->copyJointGroupAccelerations(joint_model_group_name, values);
  return values;
}

bool set_from_ik(std::shared_ptr<moveit::core::RobotState>& robot_state, const std::string& joint_model_group_name,
                 py::object& geometry_pose, const std::string& tip_name, const double timeout,
                 const moveit::core::GroupStateValidityCallbackFn& constraints,
                 const kinematics::KinematicsQueryOptions& options)
{
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(joint_model_group_name);

  geometry_msgs::msg::Pose pose_cpp = PoseToCpp(geometry_pose);

  return robot_state->setFromIK(joint_model_group, pose_cpp, tip_name, timeout, constraints, options);
}

Eigen::MatrixXd get_jacobian(std::shared_ptr<moveit::core::RobotState>& robot_state,
                             const std::string& joint_model_group_name, const Eigen::Vector3d& reference_point_position)
{
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(joint_model_group_name);
  return robot_state->getJacobian(joint_model_group, reference_point_position);
}

Eigen::MatrixXd get_jacobian(std::shared_ptr<moveit::core::RobotState>& robot_state,
                             const std::string& joint_model_group_name, const std::string link_model_name,
                             const Eigen::Vector3d& reference_point_position, bool use_quaternion_representation)
{
  Eigen::MatrixXd jacobian;
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(joint_model_group_name);
  const moveit::core::LinkModel* link_model = robot_state->getLinkModel(link_model_name);
  robot_state->getJacobian(joint_model_group, link_model, reference_point_position, jacobian,
                           use_quaternion_representation);
  return jacobian;
}

bool set_to_default_values(std::shared_ptr<moveit::core::RobotState>& robot_state,
                           const std::string& joint_model_group_name, const std::string& state_name)

{
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(joint_model_group_name);
  return robot_state->setToDefaultValues(joint_model_group, state_name);
}
