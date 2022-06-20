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
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

#include "planning_scene_monitor.h"

namespace moveit_py
{
namespace bind_planning_scene_monitor
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_py.bind_planning_scene_monitor");

LockedPlanningSceneContextManagerRO
read_only(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
{
  const planning_scene_monitor::LockedPlanningSceneRO* ls_ro =
      new planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor);
  return LockedPlanningSceneContextManagerRO(planning_scene_monitor, ls_ro);
};

LockedPlanningSceneContextManagerRW
read_write(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
{
  const planning_scene_monitor::LockedPlanningSceneRW* ls_rw =
      new planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor);
  return LockedPlanningSceneContextManagerRW(planning_scene_monitor, ls_rw);
};

const planning_scene::PlanningSceneConstPtr& LockedPlanningSceneContextManagerRO::locked_planning_scene_ro_enter_() const
{
  return static_cast<const planning_scene_monitor::PlanningSceneMonitor*>(planning_scene_monitor_.get())
      ->getPlanningScene();
}

const planning_scene::PlanningScenePtr& LockedPlanningSceneContextManagerRW::locked_planning_scene_rw_enter_()
{
  return planning_scene_monitor_->getPlanningScene();
}

void LockedPlanningSceneContextManagerRO::locked_planning_scene_ro_exit_(const py::object& type,
                                                                         const py::object& value,
                                                                         const py::object& traceback)
{
  delete ls_ro_;
}

void LockedPlanningSceneContextManagerRW::locked_planning_scene_rw_exit_(const py::object& type,
                                                                         const py::object& value,
                                                                         const py::object& traceback)
{
  delete ls_rw_;
}

void apply_planning_scene(std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>& planning_scene_monitor,
                          py::object& planning_scene)
{
  // convert python object to C++ object (use serialization for now)
  moveit_msgs::msg::PlanningScene planning_scene_cpp;

  // TODO (peterdavidfagan): replace with copy method which is much faster
  py::module_ rclpy_serialization = py::module::import("rclpy.serialization");
  py::bytes serialized_msg = rclpy_serialization.attr("serialize_message")(planning_scene);
  deserializeMsg(serialized_msg, planning_scene_cpp);

  // lock planning scene
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor);
    scene->usePlanningSceneMsg(planning_scene_cpp);
  }
}

// provide the option to perform a service call when applying planning scene (feature from planning_scene_interface)
// bool apply_planning_scene_service(const moveit_msgs::msg::PlanningScene& planning_scene)
//{
//  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
//  moveit_msgs::srv::ApplyPlanningScene::Response::SharedPtr response;
//  request->scene = planning_scene;
//
//  auto res = apply_planning_scene_service_->async_send_request(request);
//  if (rclcpp::spin_until_future_complete(node_, res) != rclcpp::FutureReturnCode::SUCCESS)
//  {
//    RCLCPP_WARN(LOGGER, "Failed to call ApplyPlanningScene service");
//  }
//  response = res.get();
//  return response->success;
//}

}  // namespace bind_planning_scene_monitor
}  // namespace moveit_py
