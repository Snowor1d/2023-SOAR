#!/usr/bin/env python

################################################################################
#
# Copyright (c) 2018, PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
################################################################################

"""
Example to launch a sensor_combined listener node.

.. seealso::
    https://index.ros.org/doc/ros2/Launch-system/
"""
import os
import sys
import time
import unittest




import launch.actions
import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import Node

import launch_testing.actions
import launch_testing.markers

from launch.actions import TimerAction

from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
     return LaunchDescription([
          
        launch.actions.TimerAction(
            period=1.0,
            actions=[
                launch_ros.actions.Node(
                    package = 'px4_ros_com',
                    executable='first_offboard',
                    name='first_offboard')
                
            ]
        ),

        launch.actions.TimerAction(
            period=10.0,
            actions=[
                launch_ros.actions.Node(
                    package = 'px4_ros_com',
                    executable='second_offboard',
                    name='second_offboard')
                
            ]
        ),

        launch.actions.TimerAction(
            period=19.0,
            actions = [
                 launch_ros.actions.Node(
                    package = 'px4_ros_com',
                    executable = 'third_offboard',
                    name = 'third_offboard')
                 
            ]
        )
    
    
    
    
    
    ])


    

