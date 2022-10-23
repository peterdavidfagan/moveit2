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


#include <pybind11/pybind11.h>
#include <copy_ros_msg.h>
#include <serialize_ros_msg.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace py = pybind11;

namespace moveit_py
{
namespace bind_planning_scene_monitor
{

class LockedPlanningSceneContextManagerRW
{
public:
  const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  const planning_scene_monitor::LockedPlanningSceneRW* ls_rw_;

  LockedPlanningSceneContextManagerRW(const planning_scene_monitor::PlanningSceneMonitorPtr& psm,
                                      const planning_scene_monitor::LockedPlanningSceneRW* ls_rw)
    : planning_scene_monitor_(psm), ls_rw_(ls_rw)
  {
  }

  const planning_scene::PlanningScenePtr& locked_planning_scene_rw_enter_();

  void locked_planning_scene_rw_exit_(const py::object& type, const py::object& value, const py::object& traceback);
};

class LockedPlanningSceneContextManagerRO
{
public:
  const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  const planning_scene_monitor::LockedPlanningSceneRO* ls_ro_;

  LockedPlanningSceneContextManagerRO(const planning_scene_monitor::PlanningSceneMonitorPtr& psm,
                                      const planning_scene_monitor::LockedPlanningSceneRO* ls_ro)
    : planning_scene_monitor_(psm), ls_ro_(ls_ro)
  {
  }

  const planning_scene::PlanningSceneConstPtr& locked_planning_scene_ro_enter_() const;

  void locked_planning_scene_ro_exit_(const py::object& type, const py::object& value, const py::object& traceback);
};

// class LockedPlanningSceneContextManager
//{
// public:
//   const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
//   const planning_scene_monitor::LockedPlanningSceneRO* ls_ro_;
//   const planning_scene_monitor::LockedPlanningSceneRW* ls_rw_;
//   bool read_only_;

//  LockedPlanningSceneContextManager(const planning_scene_monitor::PlanningSceneMonitorPtr& psm,
//                                    const planning_scene_monitor::LockedPlanningSceneRO* ls_ro, bool read_only)
//    : planning_scene_monitor_(psm), ls_ro_(ls_ro), ls_rw_(nullptr), read_only_(read_only)
//  {
//  }

//  LockedPlanningSceneContextManager(const planning_scene_monitor::PlanningSceneMonitorPtr& psm,
//                                    const planning_scene_monitor::LockedPlanningSceneRW* ls_rw, bool read_only)
//    : planning_scene_monitor_(psm), ls_ro_(nullptr), ls_rw_(ls_rw), read_only_(read_only)
//  {
//  }
//};

LockedPlanningSceneContextManagerRW
read_write(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

LockedPlanningSceneContextManagerRO
read_only(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

// const planning_scene::PlanningScenePtr& locked_planning_scene_enter_(LockedPlanningSceneContextManager& context_manager);

// void locked_planning_scene_exit_(LockedPlanningSceneContextManager& context_manager, const py::object& type,
//                                  const py::object& value, const py::object& traceback);

void apply_planning_scene(std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>& planning_scene_monitor,
                          py::object& planning_scene);

void init_planning_scene_monitor(py::module &m);
}  // namespace bind_planning_scene_monitor
}  // namespace moveit_py
