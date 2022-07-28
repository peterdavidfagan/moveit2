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
#include <copy_ros_msg.h>
#include <serialize_ros_msg.h>

#include "moveit_ros/planning_interface/planning_scene_interface.h"

namespace py = pybind11;

PYBIND11_MODULE(planning_interface, m)
{
  m.doc() = "A module containing methods to interface with a planning scene.";

  // PlanningSceneInterface
  py::class_<moveit::planning_interface::PlanningSceneInterface,
             std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>>(m, "PlanningSceneInterface")
      .def(py::init<const std::string&, bool>(), py::arg("ns") = "", py::arg("wait") = true,
           py::return_value_policy::reference)
      .def("get_known_object_names", &moveit::planning_interface::PlanningSceneInterface::getKnownObjectNames,
           py::arg("with_type") = false, py::return_value_policy::move)
      .def("get_known_object_names_in_roi",
           py::overload_cast<double, double, double, double, double, double, bool>(
               &moveit::planning_interface::PlanningSceneInterface::getKnownObjectNamesInROI),
           py::arg("min_x"), py::arg("min_y"), py::arg("min_z"), py::arg("max_x"), py::arg("max_y"), py::arg("max_z"),
           py::arg("with_type") = false, py::return_value_policy::move)
      .def("get_object_poses", &get_object_poses, py::arg("object_ids"), py::return_value_policy::move)
      //.def("get_attached_objects", &get_attached_objects, py::arg("object_ids"), py::return_value_policy::move)
      //.def("add_collision_objects", &add_collision_objects, py::arg("objects_ids"), py::return_value_policy::move)
      .def("remove_collision_objects", &moveit::planning_interface::PlanningSceneInterface::removeCollisionObjects,
           py::arg("object_ids"), py::return_value_policy::move)
      //.def("apply_planning_scene", &apply_planning_scene, py::arg("planning_scene"))
      .def("apply_collision_object", &apply_collision_object, py::arg("collision_object"));
}
