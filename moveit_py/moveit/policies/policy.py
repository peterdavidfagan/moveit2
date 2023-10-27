# Software License Agreement (BSD License)
#
# Copyright (c) 2022, Peter David Fagan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Peter David Fagan
""" Definition of an abstract base class for policy deployment. """

from abc import ABC, abstractmethod

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image

from std_srvs.srv import SetBool

import message_filters

class Policy(ABC, Node):
    """An abstract base class for deploying learnt policies."""
    
    def __init__(self, params, node_name="policy_node"):
        """Initialise the policy."""
        super().__init__(node_name)
        self.logger = self.get_logger()        
        self.param_listener = params.ParamListener(self)
        self.params = self.param_listener.get_params()
        
        # status of policy rollout
        self.active = False
        
        # register sensor topics
        self.register_sensors()
        
        # register controller topics
        self.register_controller()
        
        # TODO: create service for pausing and resuming policy execution
        self.activate_policy = self.create_service( 
            SetBool,
            "activate_policy",
            self.activate_policy,
            )

    def get_sensor_msg_type(self, msg_type):
        if msg_type == "sensor_msgs/Image":
            return Image
        else:
            raise ValueError("Sensor type {} not supported.".format(msg_type))

    def get_controller_msg_type(self, msg_type):
        if msg_type == "geometry_msgs/PoseStamped":
            return PoseStamped
        elif msg_type == "geometry_msgs/Twist":
            return Twist
        else:
            raise ValueError("Controller type {} not supported.".format(msg_type))

    def register_sensors(self):
        """Register the topics to listen to for sensor data."""
        self.sensor_subs = []
        for sensor_idx in range(self.params.num_sensors):
            # TODO: refactor this section  !!!
            sensor_params = self.get_parameters_by_prefix("sensor{}".format(sensor_idx+1))
            self.sensor_subs.append(message_filters.Subscriber(
                    self, 
                    self.get_sensor_msg_type(sensor_params["type"].value), 
                    str(sensor_params["topic"].value),
                    #int(sensor_params["qos"].value),
                    ))

        # create a synchronizer for the sensor topics
        self.sensor_sync = message_filters.ApproximateTimeSynchronizer(
            self.sensor_subs,
            self.params.sensor_queue,
            self.params.sensor_slop,
            )
        
        # register model forward pass as callback
        self.sensor_sync.registerCallback(self.forward)

    def register_controller(self):
        """Register the topic to publish actions to."""
        self.controller_pub = self.create_publisher(
            self.get_controller_msg_type(self.params.controller1.type),
            self.params.controller1.topic,
            self.params.controller1.qos,
            )

    def activate_policy(self, request, response):
        """Activate the policy."""
        self.active = request.data
        return response

    @abstractmethod
    def forward():
        """Perform a forward pass of the policy."""
        pass

