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

#include <list>
#include "copy_ros_msg.h"

// Ros Message Copy Definitions (Note: copying faster than serialize/deserialize)

geometry_msgs::msg::PoseStamped PoseStampedToCpp(py::object pose_stamped)
{
  // recreate instance in C++ using python object data
  geometry_msgs::msg::PoseStamped pose_stamped_cpp;
  pose_stamped_cpp.header.frame_id = pose_stamped.attr("header").attr("frame_id").cast<std::string>();
  pose_stamped_cpp.pose.orientation.w = pose_stamped.attr("pose").attr("orientation").attr("w").cast<double>();
  pose_stamped_cpp.pose.position.x = pose_stamped.attr("pose").attr("position").attr("x").cast<double>();
  pose_stamped_cpp.pose.position.y = pose_stamped.attr("pose").attr("position").attr("y").cast<double>();
  pose_stamped_cpp.pose.position.z = pose_stamped.attr("pose").attr("position").attr("z").cast<double>();

  return pose_stamped_cpp;
}

geometry_msgs::msg::Pose PoseToCpp(py::object pose)
{
  // recreate instance in C++ using python object data
  geometry_msgs::msg::Pose pose_cpp;
  pose_cpp.orientation.w = pose.attr("orientation").attr("w").cast<double>();
  pose_cpp.position.x = pose.attr("position").attr("x").cast<double>();
  pose_cpp.position.y = pose.attr("position").attr("y").cast<double>();
  pose_cpp.position.z = pose.attr("position").attr("z").cast<double>();

  return pose_cpp;
}

py::object PoseToPy(geometry_msgs::msg::Pose pose)
{
  // recreate instance in Python using C++ object data
  py::object pose_py = py::module_::import("geometry_msgs.msg").attr("Pose")();

  pose_py.attr("orientation").attr("w") = pose.orientation.w;
  pose_py.attr("position").attr("x") = pose.position.x;
  pose_py.attr("position").attr("y") = pose.position.y;
  pose_py.attr("position").attr("z") = pose.position.z;

  return pose_py;
}

geometry_msgs::msg::Point PointToCpp(py::object point)
{
  // recreate instance in C++ using python object data
  geometry_msgs::msg::Point point_cpp;
  point_cpp.x = point.attr("x").cast<double>();
  point_cpp.y = point.attr("y").cast<double>();
  point_cpp.z = point.attr("z").cast<double>();

  return point_cpp;
}

geometry_msgs::msg::Vector3 Vector3ToCpp(py::object vector3)
{
  // recreate instance in C++ using python object data
  geometry_msgs::msg::Vector3 vector3_cpp;
  vector3_cpp.x = vector3.attr("x").cast<double>();
  vector3_cpp.y = vector3.attr("y").cast<double>();
  vector3_cpp.z = vector3.attr("z").cast<double>();

  return vector3_cpp;
}

geometry_msgs::msg::Quaternion QuaternionToCpp(py::object quaternion)
{
  // recreate instance in C++ using python object data
  geometry_msgs::msg::Quaternion quaternion_cpp;
  quaternion_cpp.w = quaternion.attr("w").cast<double>();
  quaternion_cpp.x = quaternion.attr("x").cast<double>();
  quaternion_cpp.y = quaternion.attr("y").cast<double>();
  quaternion_cpp.z = quaternion.attr("z").cast<double>();

  return quaternion_cpp;
}

shape_msgs::msg::SolidPrimitive SolidPrimitiveToCpp(py::object primitive)
{
  // recreate instance in C++ using python object data
  shape_msgs::msg::SolidPrimitive primitive_cpp;
  primitive_cpp.type = primitive.attr("type").cast<int>();
  int num_dimensions = primitive.attr("dimensions").attr("__len__")().cast<int>();
  for (int j = 0; j < num_dimensions; j++)
  {
    primitive_cpp.dimensions.push_back(primitive.attr("dimensions").attr("__getitem__")(j).cast<double>());
  }

  return primitive_cpp;
}

shape_msgs::msg::MeshTriangle MeshTriangleToCpp(py::object mesh_triangle)
{
  // recreate instance in C++ using python object data
  shape_msgs::msg::MeshTriangle mesh_triangle_cpp;
  mesh_triangle_cpp.vertex_indices[0] = mesh_triangle.attr("vertex_indices").attr("__getitem__")(0).cast<int>();
  mesh_triangle_cpp.vertex_indices[1] = mesh_triangle.attr("vertex_indices").attr("__getitem__")(1).cast<int>();
  mesh_triangle_cpp.vertex_indices[2] = mesh_triangle.attr("vertex_indices").attr("__getitem__")(2).cast<int>();

  return mesh_triangle_cpp;
}

shape_msgs::msg::Mesh MeshToCpp(py::object mesh)
{
  // recreate instance in C++ using python object data
  shape_msgs::msg::Mesh mesh_cpp;
  mesh_cpp.vertices.resize(mesh.attr("vertices").attr("__len__")().cast<int>());
  for (int j = 0; j < mesh.attr("vertices").attr("__len__")().cast<int>(); j++)
  {
    mesh_cpp.vertices.push_back(PointToCpp(mesh.attr("vertices").attr("__getitem__")(j)));
  }
  mesh_cpp.triangles.resize(mesh.attr("triangles").attr("__len__")().cast<int>());
  for (int j = 0; j < mesh.attr("triangles").attr("__len__")().cast<int>(); j++)
  {
    mesh_cpp.triangles.push_back(MeshTriangleToCpp(mesh.attr("triangles").attr("__getitem__")(j)));
  }

  return mesh_cpp;
}

moveit_msgs::msg::BoundingVolume BoundingVolumeToCpp(py::object bounding_volume)
{
  // recreate instance in C++ using python object data
  moveit_msgs::msg::BoundingVolume bounding_volume_cpp;

  // primitives
  for (int j = 0; j < bounding_volume.attr("primitives").attr("__len__")().cast<int>(); j++)
  {
    bounding_volume_cpp.primitives.push_back(
        SolidPrimitiveToCpp(bounding_volume.attr("primitives").attr("__getitem__")(j)));
  }

  // primitive poses
  for (int j = 0; j < bounding_volume.attr("primitive_poses").attr("__len__")().cast<int>(); j++)
  {
    bounding_volume_cpp.primitive_poses.push_back(
        PoseToCpp(bounding_volume.attr("primitive_poses").attr("__getitem__")(j)));
  }

  // meshes
  for (int j = 0; j < bounding_volume.attr("meshes").attr("__len__")().cast<int>(); j++)
  {
    bounding_volume_cpp.meshes.push_back(MeshToCpp(bounding_volume.attr("meshes").attr("__getitem__")(j)));
  }

  // mesh poses
  for (int j = 0; j < bounding_volume.attr("mesh_poses").attr("__len__")().cast<int>(); j++)
  {
    bounding_volume_cpp.mesh_poses.push_back(PoseToCpp(bounding_volume.attr("mesh_poses").attr("__getitem__")(j)));
  }

  return bounding_volume_cpp;
}

moveit_msgs::msg::JointConstraint JointConstraintToCpp(py::object joint_constraint)
{
  // recreate instance in C++ using python object data
  moveit_msgs::msg::JointConstraint joint_constraint_cpp;
  joint_constraint_cpp.joint_name = joint_constraint.attr("joint_name").cast<std::string>();
  joint_constraint_cpp.position = joint_constraint.attr("position").cast<double>();
  joint_constraint_cpp.tolerance_above = joint_constraint.attr("tolerance_above").cast<double>();
  joint_constraint_cpp.tolerance_below = joint_constraint.attr("tolerance_below").cast<double>();
  joint_constraint_cpp.weight = joint_constraint.attr("weight").cast<double>();

  return joint_constraint_cpp;
}

moveit_msgs::msg::PositionConstraint PositionConstraintToCpp(py::object position_constraint)
{
  // recreate instance in C++ using python object data
  moveit_msgs::msg::PositionConstraint position_constraint_cpp;
  position_constraint_cpp.header.frame_id = position_constraint.attr("header").attr("frame_id").cast<std::string>();
  position_constraint_cpp.link_name = position_constraint.attr("link_name").cast<std::string>();
  position_constraint_cpp.target_point_offset = Vector3ToCpp(position_constraint.attr("target_point_offset"));
  position_constraint_cpp.constraint_region = BoundingVolumeToCpp(position_constraint.attr("constraint_region"));
  position_constraint_cpp.weight = position_constraint.attr("weight").cast<double>();

  return position_constraint_cpp;
}

moveit_msgs::msg::OrientationConstraint OrientationConstraintToCpp(py::object orientation_constraint)
{
  // recreate instance in C++ using python object data
  moveit_msgs::msg::OrientationConstraint orientation_constraint_cpp;
  orientation_constraint_cpp.header.frame_id =
      orientation_constraint.attr("header").attr("frame_id").cast<std::string>();
  orientation_constraint_cpp.link_name = orientation_constraint.attr("link_name").cast<std::string>();
  orientation_constraint_cpp.orientation = QuaternionToCpp(orientation_constraint.attr("target_quaternion"));
  orientation_constraint_cpp.absolute_x_axis_tolerance =
      orientation_constraint.attr("absolute_x_axis_tolerance").cast<double>();
  orientation_constraint_cpp.absolute_y_axis_tolerance =
      orientation_constraint.attr("absolute_y_axis_tolerance").cast<double>();
  orientation_constraint_cpp.absolute_z_axis_tolerance =
      orientation_constraint.attr("absolute_z_axis_tolerance").cast<double>();
  orientation_constraint_cpp.parameterization = orientation_constraint.attr("parameterization").cast<int>();
  orientation_constraint_cpp.weight = orientation_constraint.attr("weight").cast<double>();

  return orientation_constraint_cpp;
}

moveit_msgs::msg::VisibilityConstraint VisibilityConstraintToCpp(py::object visibility_constraint)
{
  // recreate instance in C++ using python object data
  moveit_msgs::msg::VisibilityConstraint visibility_constraint_cpp;
  visibility_constraint_cpp.target_radius = visibility_constraint.attr("target_radius").cast<double>();
  visibility_constraint_cpp.target_pose = PoseStampedToCpp(visibility_constraint.attr("target_pose"));
  visibility_constraint_cpp.cone_sides = visibility_constraint.attr("cone_sides").cast<int>();
  visibility_constraint_cpp.sensor_pose = PoseStampedToCpp(visibility_constraint.attr("sensor_pose"));
  visibility_constraint_cpp.max_view_angle = visibility_constraint.attr("max_view_angle").cast<double>();
  visibility_constraint_cpp.max_range_angle = visibility_constraint.attr("max_range_angle").cast<double>();
  visibility_constraint_cpp.sensor_view_direction = visibility_constraint.attr("sensor_view_direction").cast<int>();
  visibility_constraint_cpp.weight = visibility_constraint.attr("weight").cast<double>();

  return visibility_constraint_cpp;
}

moveit_msgs::msg::CollisionObject CollisionObjectToCpp(py::object collision_object)
{
  //  recreate instance in C++ using python object data
  moveit_msgs::msg::CollisionObject collision_object_cpp;

  // header
  collision_object_cpp.header.frame_id = collision_object.attr("header").attr("frame_id").cast<std::string>();

  // object pose
  collision_object_cpp.pose = PoseToCpp(collision_object.attr("pose"));

  // object id
  collision_object_cpp.id = collision_object.attr("id").cast<std::string>();

  // object type
  collision_object_cpp.type.key = collision_object.attr("type").attr("key").cast<std::string>();
  collision_object_cpp.type.db = collision_object.attr("type").attr("db").cast<std::string>();

  // iterate through python list creating C++ vector of primitives
  int num_primitives = collision_object.attr("primitives").attr("__len__")().cast<int>();
  for (int i = 0; i < num_primitives; i++)
  {
    py::object primitive = collision_object.attr("primitives").attr("__getitem__")(i);
    auto primitive_cpp = SolidPrimitiveToCpp(primitive);
    collision_object_cpp.primitives.push_back(primitive_cpp);
  }

  // iterate through python list creating C++ vector of primitive poses
  int num_primitives_poses = collision_object.attr("primitive_poses").attr("__len__")().cast<int>();
  for (int i = 0; i < num_primitives_poses; i++)
  {
    py::object primitive_pose = collision_object.attr("primitive_poses").attr("__getitem__")(i);
    auto primitive_pose_cpp = PoseToCpp(primitive_pose);
    collision_object_cpp.primitive_poses.push_back(primitive_pose_cpp);
  }

  // iterate through python list creating C++ vector of meshes
  int num_meshes = collision_object.attr("meshes").attr("__len__")().cast<int>();
  for (int i = 0; i < num_meshes; i++)
  {
    // TODO (peterdavidfagan):  implement mesh conversion
    py::object mesh = collision_object.attr("meshes").attr("__getitem__")(i);
    auto mesh_cpp = MeshToCpp(mesh);
    collision_object_cpp.meshes.push_back(mesh_cpp);
  }

  // iterate through python list creating C++ vector of mesh poses
  int num_mesh_poses = collision_object.attr("mesh_poses").attr("__len__")().cast<int>();
  for (int i = 0; i < num_mesh_poses; i++)
  {
    py::object mesh_pose = collision_object.attr("mesh_poses").attr("__getitem__")(i);
    auto mesh_pose_cpp = PoseToCpp(mesh_pose);
    collision_object_cpp.mesh_poses.push_back(mesh_pose_cpp);
  }

  // operation
  collision_object_cpp.operation = collision_object.attr("operation").cast<char>();

  return collision_object_cpp;
}

moveit_msgs::msg::Constraints ConstraintsToCpp(py::object constraints)
{
  // recreate instance in C++ using python object data
  moveit_msgs::msg::Constraints constraints_cpp;

  // iterate through python list creating C++ vector of joint constraints
  int num_joint_constraints = constraints.attr("joint_constraints").attr("__len__")().cast<int>();
  for (int i = 0; i < num_joint_constraints; i++)
  {
    py::object joint_constraint = constraints.attr("__getitem__")(i);
    auto joint_constraint_cpp = JointConstraintToCpp(joint_constraint);
    constraints_cpp.joint_constraints.push_back(joint_constraint_cpp);
  }

  // iterate through python list creating C++ vector of position constraints
  int num_position_constraints = constraints.attr("position_constraints").attr("__len__")().cast<int>();
  for (int i = 0; i < num_position_constraints; i++)
  {
    py::object position_constraint = constraints.attr("__getitem__")(i);
    auto position_constraint_cpp = PositionConstraintToCpp(position_constraint);
    constraints_cpp.position_constraints.push_back(position_constraint_cpp);
  }

  // iterate through python list creating C++ vector of orientation constraints
  int num_orientation_constraints = constraints.attr("orientation_constraints").attr("__len__")().cast<int>();
  for (int i = 0; i < num_orientation_constraints; i++)
  {
    py::object orientation_constraint = constraints.attr("orientation_constraints").attr("__getitem__")(i);
    auto orientation_constraint_cpp = OrientationConstraintToCpp(orientation_constraint);
    constraints_cpp.orientation_constraints.push_back(orientation_constraint_cpp);
  }

  // iterate through python list creating C++ vector of visibility constraints
  int num_visibility_constraints = constraints.attr("visibility_constraints").attr("__len__")().cast<int>();
  for (int i = 0; i < num_visibility_constraints; i++)
  {
    py::object visibility_constraint = constraints.attr("visibility_constraints").attr("__getitem__")(i);
    auto visibility_constraint_cpp = VisibilityConstraintToCpp(visibility_constraint);
    constraints_cpp.visibility_constraints.push_back(visibility_constraint_cpp);
  }

  return constraints_cpp;
}

// TODO (peterdavidfagan): think of a more clever way to perform type cast of ros msgs from python -> cpp
// it is possible to serialize/deserialize but this is slower than just copying data.

// define mapping from string to pointer creation for standard types
// auto convert_message(py::object& python_msg)
//{
// construct string outlining the message type
//  std::string module_name = python_msg.attr("__module__").cast<std::string>();
//  std::string module_name_parsed = module_name.substr(0, module_name.find('.'));
//  std::string msg_name = python_msg.attr("__class__").attr("__name__").cast<std::string>();
//  std::string msg_type_str = module_name_parsed + "::" + "msg" + "::" + msg_name;

// Create C++ equivalent of message type
//  auto rosmsg = rosmsg_map[msg_type_str].create_message();

// iterate through each member of python class to assign members to C++
// object
//  auto field_map = python_msg.attr("get_fields_and_field_types")().cast<std::map<std::string, std::string>>();

// iterate through all elements of created message
//  for (auto& field : field_map)
//  {
// if the member is a standard data type cast to C++ equivalent
// type and assign as variable in C++ object
//    if (standard_data_types.find(field.second) != standard_data_types.end())
//    {
// assign variable in C++ object
//      rosmsg[field.first.c_str()] = python_msg.attr(field.first.c_str()).cast<standard_data_types[field.second]>();
//    }
// else create C++ object of the same type as Python class and
// recursively call this function on the member
//    else
//    {
// create C++ object of the same type as Python class
//      rosmsg[field.first] = convert_message(python_msg.attr(field.first);
//    }
//  }

//  return rosmsg;
//}_
