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
#include <serialize_ros_msg.h>
#include "planning_scene_interface.h"

bool apply_collision_object(std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>& planning_scene_interface,
                            py::object& collision_object_msg)
{
  // convert python object to ROS message
  moveit_msgs::msg::CollisionObject collision_object_cpp = CollisionObjectToCpp(collision_object_msg);

  // apply collision object
  return planning_scene_interface->applyCollisionObject(collision_object_cpp);
}

py::dict get_object_poses(std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_interface,
                          std::vector<std::string> object_ids)
{
  auto object_poses = planning_interface->getObjectPoses(object_ids);
  py::dict object_poses_py;
  for (auto& object_pose : object_poses)
  {
    object_poses_py[py::str(object_pose.first)] = PoseToPy(object_pose.second);
  }

  return object_poses_py;
}
