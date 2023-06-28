#!/usr/bin/env python3
# -*- coding: utf-8 -*-
############################################################################
#
#   Copyright (C) 2023 SOAR-Snowor1d. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleOdometry

import math
import numpy as np


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/offboard_control_mode/in', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/trajectory_setpoint/in', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/vehicle_command/in', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/vehicle_local_position/out', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/vehicle_status/out', self.vehicle_status_callback, qos_profile)
        self.vehicle_odom_subscriber = self.create_subscription(VehicleOdometry, '/fmu/vehicle_odometry/out', self.vehicle_odom_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.vehicle_odom = VehicleOdometry()


        self.waypoint_list = [[0,0,-5], [8,0,-5], [2,4,-5], [4,0,-5], [0, 0, 0]]
        self.waypoint_velocity = [2, 0.6, 1.4, 3, 2]
        self.waypoint_yaw = [0, 1, 1, 1, 0]
        self.waypoint_num = 5
        self.waypoint_count = 0
        self.waypoint_range = 0.5
        self.correction_range = 1.5
        self.previous_waypoint = [0,0,0]
        self.wait_in_waypoint = 0
        self.setpoint_mode = False
        self.previous_yaw = 0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def vehicle_odom_callback(self, vehicle_odom):
        self.vehicle_odom = vehicle_odom
        

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self, is_p):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        if is_p:
            msg.velocity = False
            msg.position = True 

        else:
            msg.velocity = True
            msg.position = False 
        
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)


    def publish_position_setpoint(self, t_x: float, t_y: float, t_z: float, v:float, yaw:float):
        """Publish the trajectory setpoint."""
        x = self.previous_waypoint[0]
        y = self.previous_waypoint[1]
        
        msg = TrajectorySetpoint()
        msg.x = t_x
        msg.y = t_y
        msg.z = t_z
        msg.yaw = float(self.previous_yaw)
      
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        #self.get_logger().info(f"Publishing position setpoints {[t_x, t_y, t_z]}")
        #self.get_logger().info(f"intended vx vy vz {[msg.vx, msg.vy, msg.vz]}")
        #self.get_logger().info(f"odom vx vy vz {[self.vehicle_odom.vx, self.vehicle_odom.vy, self.vehicle_odom.vz]}")
        #self.get_logger().info(f"Controlled by position..")

    def publish_velocity_setpoint(self, t_x: float, t_y: float, t_z:float, v:float, yaw:float):
        pi = math.pi
        x = self.previous_waypoint[0]
        y = self.previous_waypoint[1]
        z = self.previous_waypoint[2]
        #x = self.vehicle_odom.x
        #y = self.vehicle_odom.y
        #z = self.vehicle_odom.z
        
        msg = TrajectorySetpoint() 
        msg.x = np.nan
        msg.y = np.nan
        msg.z = np.nan

        msg.vx = v * ((t_x-x)/(math.sqrt(math.pow(t_x-x,2)+math.pow(t_y-y,2)+math.pow(t_z-z,2))))
        msg.vy = v * ((t_y-y)/(math.sqrt(math.pow(t_x-x,2)+math.pow(t_y-y,2)+math.pow(t_z-z,2))))
        msg.vz = v * ((t_z-z)/(math.sqrt(math.pow(t_x-x,2)+math.pow(t_y-y,2)+math.pow(t_z-z,2))))
    
        diff_x = t_x-x
        diff_y = t_y-y
        if(yaw!=0):
            if(diff_x>0 and diff_y>0):
                msg.yaw = math.atan(diff_y/diff_x)
            elif(diff_x<0 and diff_y>0):
                msg.yaw = pi - math.atan(abs(diff_y/diff_x))
            elif(diff_x<0 and diff_y<0):
                msg.yaw = -(pi - math.atan(abs(diff_y/diff_x)))
            elif(diff_x>0 and diff_y<0):
                msg.yaw = -(math.atan(abs(diff_y/diff_x)))
            elif(diff_x==0):
                if(diff_y==0):
                    msg.yaw = float(self.previous_yaw)
                elif(diff_y>0):
                    msg.yaw = pi/2
                else:
                    msg.yaw = -pi/2
            elif(diff_x<0):
                if(diff_y==0):
                    msg.yaw = pi
            elif(diff_x>0):
                if(diff_y==0):
                    msg.yaw = float(0)
            self.previous_yaw = msg.yaw
        else:
            msg.yaw = float(self.previous_yaw)

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        #self.get_logger().info(f"Controlled by velocity..")
        #self.get_logger().info(f"Publishing position setpoints {[t_x, t_y, t_z]}")
        #self.get_logger().info(f"intended vx vy vz {[msg.vx, msg.vy, msg.vz]}")
        #self.get_logger().info(f"odom x y z {[self.vehicle_odom.x, self.vehicle_odom.y, self.vehicle_odom.z]}")
        #self.get_logger().info(f"odom vx vy vz {[self.vehicle_odom.vx, self.vehicle_odom.vy, self.vehicle_odom.vz]}")

    def compensation_path_with_odom(self):
        self.previous_waypoint[0] = self.vehicle_odom.x
        self.previous_waypoint[1] = self.vehicle_odom.y
        self.previous_waypoint[2] = self.vehicle_odom.z

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        #self.get_logger().info(f"local position {[self.vehicle_odom.x, self.vehicle_odom.y, self.vehicle_odom.z]}")
        """Callback function for the timer."""
        
        x = self.waypoint_list[self.waypoint_count][0]
        y = self.waypoint_list[self.waypoint_count][1]
        z = self.waypoint_list[self.waypoint_count][2]
        self.publish_offboard_control_heartbeat_signal(True)
        distance_target = math.sqrt(math.pow(x-self.vehicle_odom.x,2)+math.pow(y-self.vehicle_odom.y,2)+math.pow((z-self.vehicle_odom.z),2))
        
        if self.offboard_setpoint_counter == 10: # timer_callback -> node publish 할때마다 실행됨. 즉, publish되는 주기만큼 timer_callback이 실행된다고 이해할 수 있다
                                                 # -> publish가 10번 되었을때 함수 실행. 주기가 1/15면 10/15초에 함수 실행 됨.   
            self.engage_offboard_mode()
            self.arm()

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if ((distance_target < self.correction_range) | (self.setpoint_mode)):
                self.setpoint_mode = True
                self.publish_offboard_control_heartbeat_signal(True)
                self.publish_position_setpoint(float(x), float(y), float(z), self.waypoint_velocity[self.waypoint_count], self.waypoint_yaw[self.waypoint_count])

            else:
                self.publish_offboard_control_heartbeat_signal(False)
                self.publish_velocity_setpoint(float(x), float(y), float(z), self.waypoint_velocity[self.waypoint_count], self.waypoint_yaw[self.waypoint_count])

       
        self.offboard_setpoint_counter += 1

        if (distance_target < self.waypoint_range) & (self.waypoint_count != (self.waypoint_num-1)):
            self.setpoint_mode = True
            self.wait_in_waypoint += 1
            if (self.wait_in_waypoint == 30):
                self.previous_waypoint[0] = self.waypoint_list[self.waypoint_count][0]
                self.previous_waypoint[1] = self.waypoint_list[self.waypoint_count][1]
                self.previous_waypoint[2] = self.waypoint_list[self.waypoint_count][2]
                
                self.get_logger().info(f"{[self.waypoint_list[self.waypoint_count][0], self.waypoint_list[self.waypoint_count][1], self.waypoint_list[self.waypoint_count][2]]}, departed !! ")

                self.waypoint_count = self.waypoint_count + 1

                self.get_logger().info(f"To {[self.waypoint_list[self.waypoint_count][0], self.waypoint_list[self.waypoint_count][1], self.waypoint_list[self.waypoint_count][2]]}, at {self.waypoint_velocity[self.waypoint_count]}m/s")
                self.wait_in_waypoint = 0
                self.setpoint_mode = False
        
        if (self.offboard_setpoint_counter % 45 == 0):  #timer_callback 주기 1/15라고 가정, 3초마다 보정 
            self.get_logger().info(f"** path correction with odom **")
            self.compensation_path_with_odom()
            
            

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)