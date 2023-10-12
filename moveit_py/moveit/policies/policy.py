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

import message_filters

class Policy(ABC, Node):
    """An abstract base class for deploying learnt policies."""
    
    def __init__(self, cfg):
        """Initialise the policy."""
        super().__init__()
        
        # status of policy rollout
        self.active = False

        # register sensor topics
        self.register_sensors_topics(cfg.sensors)
        
        # register actuator topics
        self.register_actuator_topics(cfg.actuators)
        
        # TODO: create service for pausing and resuming policy execution

    def register_sensors_topics(self, sensor_cfg):
        """Register the topics to listen to for sensor data."""
        topic_subs = []
        for sensor in sensor_cfg.sensors:
            topic_subs.append(message_filters.Subscriber(
                    self, 
                    sensor.type, 
                    sensor.topic,
                    sensor.qos,
                    ))

        # create a synchronizer for the sensor topics
        self.sensor_sync = message_filters.ApproximateTimeSynchronizer(
            topic_subs,
            sensor_cfg.queue_size,
            sensor_cfg.slop,
            )
        self.sensor_sync.registerCallback(self.forward)

    def register_actuator_topics(self, action_cfg):
        """Register the topic to publish actions to."""
        self.action_pub = self.create_publisher(
            action_cfg.type,
            action_cfg.topic,
            action_cfg.qos,
            )

    def pause_execution(self):
        """Pause the execution of the policy."""
        self.active = False

    def resume_execution(self):
        """Resume the execution of the policy."""
        self.active = True

    @abstractmethod
    def forward():
        """Perform a forward pass of the policy."""
        pass

