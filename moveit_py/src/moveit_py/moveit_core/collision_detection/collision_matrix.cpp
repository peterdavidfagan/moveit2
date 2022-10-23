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
 *   * Neither the name of PickNik Inc. nor te names of its
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

#include "collision_matrix.h"

namespace moveit_py
{
namespace bind_collision_detection
{
void init_acm(py::module &m)
{
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
}
}  // namespace bind_collision_detection
}  // namespace moveit_py
