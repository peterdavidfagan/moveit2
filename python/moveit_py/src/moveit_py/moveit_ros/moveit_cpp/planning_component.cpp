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
#include "planning_component.h"


bool set_goal(std::shared_ptr<moveit_cpp::PlanningComponent>& planning_component, py::object& pose_stamped, std::string link_name)
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
