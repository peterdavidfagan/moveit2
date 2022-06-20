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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <pybind11/pybind11.h>

namespace py = pybind11;

geometry_msgs::msg::PoseStamped PoseStampedToCpp(py::object pose_stamped);

// TODO(peterdavidfagan): consider creating typecaster
geometry_msgs::msg::Pose PoseToCpp(py::object pose);
py::object PoseToPy(geometry_msgs::msg::Pose pose);

geometry_msgs::msg::Point PointToCpp(py::object point);

geometry_msgs::msg::Vector3 Vector3ToCpp(py::object vector3);

geometry_msgs::msg::Quaternion QuaternionToCpp(py::object quaternion);

shape_msgs::msg::SolidPrimitive SolidPrimitiveToCpp(py::object primitive);

shape_msgs::msg::MeshTriangle MeshTriangleToCpp(py::object mesh_triangle);

shape_msgs::msg::Mesh MeshToCpp(py::object mesh);

moveit_msgs::msg::BoundingVolume BoundingVolumeToCpp(py::object bounding_volume);

moveit_msgs::msg::JointConstraint JointConstraintToCpp(py::object joint_constraint);

moveit_msgs::msg::PositionConstraint PositionConstraintToCpp(py::object position_constraint);

moveit_msgs::msg::OrientationConstraint OrientationConstraintToCpp(py::object orientation_constraint);

moveit_msgs::msg::VisibilityConstraint VisibilityConstraintToCpp(py::object visibility_constraint);

moveit_msgs::msg::CollisionObject CollisionObjectToCpp(py::object collision_object);

moveit_msgs::msg::Constraints ConstraintsToCpp(py::object constraints);

// TODO (peterdavidfagan): come up with a more clever way of casting python rosmsg to cpp rosmsg
