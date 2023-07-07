#!/usr/bin/env python3
# -*- coding: utf-8 -*-
############################################################################
#   version 230707 1530
#   Copyright (C) 2023 SOAR-Snowr1d. All rights reserved.
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
import sys
from SE_algorithm import SE
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleOdometry

import math
import numpy as np


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('SOAR_HAVE_CONTROL')

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

        self.waypoint_contest = [[0,0,-2], [70, 30, -2], [140, 100, -2]]
        self.waypoint_list = [[0,0,-2], [6,0,-2], [-6,0,-2], [6,0,-2], [9,4,-2], [6, 0, -2], [5, 0, -2], [6, 0, 0]]
        #                      way1     mission1     way2     mission2     way3        way1     missoin3     landing
        self.waypoint_velocity = [2, 5, 5, 5, 5, 5, 5, 2]
        self.waypoint_yaw = [0, 1, 1, 1, 1, 1, 1, 0]
        self.waypoint_num = len(self.waypoint_list)
        self.waypoint_count = 0
        self.waypoint_range = 0.4
        self.correction_range = 2.5
        self.previous_waypoint = [0,0,0]
        self.wait_in_waypoint = 0
        self.setpoint_mode = False
        self.previous_yaw = 0
        self.distance_target = 0
        self.is_go_to_center = 0
        self.stable_counter = 0
        self.stable_odom = [0, 0, 0]

        self.wait_time = 0
    
        self.is_new_go = 0
        self.is_departed = 0
        
        
        #variables for ladder
        self.is_mission_ladder = 0
        self.is_mission_delivery = 0
        self.is_mission_started = 0
        self.ladder_mission_count = 0
        self.is_ladder_mission_finished = 0
        self.is_ladder_detected = 0

        self.real_obstacle_list = [[72.5, 32.5, 1.5], [67.5, 27.5, 1.5]]
        self.waypoint_for_ladder = []
        
        self.theta = 0
        self.initial_theta = -1
        self.circle_path = 1

        # Create a timer to publish control commands
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback)
    
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
        msg.x = float(t_x)
        msg.y = float(t_y)
        msg.z = float(t_z)
        msg.yaw = float(self.previous_yaw)
 
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)


    def stable_depart_publish(self, t_x: float, t_y : float, t_z : float, v:float, yaw:float):
        pi = math.pi
        self.previous_waypoint[0] = self.vehicle_odom.x
        self.previous_waypoint[1] = self.vehicle_odom.y
        self.previous_waypoint[2] = self.vehicle_odom.z
        self.stable_counter+=1
        
        self.stable_odom[0] += self.vehicle_odom.x
        self.stable_odom[1] += self.vehicle_odom.y
        self.stable_odom[2] += self.vehicle_odom.z
        
        if(self.stable_counter % 3==0):
            self.previous_waypoint[0] = self.stable_odom[0]/3
            self.previous_waypoint[1] = self.stable_odom[1]/3
            self.previous_waypoint[2] = self.stable_odom[2]/3
            self.stable_odom = [0, 0, 0]

        if(self.distance_target<self.waypoint_range):
            if(v<0.65):
                self.publish_velocity_setpoint(t_x, t_y, t_z, v*(self.distance_target/self.correction_range), 0)
                return
            self.publish_velocity_setpoint(t_x, t_y, t_z, v*(pow(self.distance_target,2)/pow(self.correction_range,2)),0)
            return
        elif(v<0.65):
            self.publish_velocity_setpoint(t_x, t_y, t_z, v*(math.sqrt(self.distance_target)/math.sqrt(self.correction_range)), 0)
            return 
        self.publish_velocity_setpoint(t_x, t_y, t_z, v*(self.distance_target/self.correction_range), 0)

    def publish_velocity_setpoint(self, t_x: float, t_y: float, t_z:float, v:float, yaw:float):
        pi = math.pi
        x = self.previous_waypoint[0]
        y = self.previous_waypoint[1]
        z = self.previous_waypoint[2]

        
        msg = TrajectorySetpoint() 
        msg.x = np.nan
        msg.y = np.nan
        msg.z = np.nan
        
        if(t_x==x and t_y == y and t_z==z):
            msg.vx = float(0)
            msg.vy = float(0)
            msg.vz = float(0)
        else:
            msg.vx = v * ((t_x-x)/(math.sqrt(math.pow(t_x-x,2)+math.pow(t_y-y,2)+math.pow(t_z-z,2))))
            msg.vy = v * ((t_y-y)/(math.sqrt(math.pow(t_x-x,2)+math.pow(t_y-y,2)+math.pow(t_z-z,2))))
            msg.vz = v * ((t_z-z)/(math.sqrt(math.pow(t_x-x,2)+math.pow(t_y-y,2)+math.pow(t_z-z,2))))
    
        diff_x = t_x-x
        diff_y = t_y-y
        if(yaw!=0):
            msg.yaw = math.atan2(diff_y,diff_x)
            self.previous_yaw = msg.yaw
        else:
            msg.yaw = float(self.previous_yaw)

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

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

    def goto_waypoint(self, to_x, to_y, to_z, velo, yaw):
        if(self.is_new_go == 1):
            self.get_logger().info(f"To {[to_x, to_y, to_z]}, at {velo}m/s")
            self.is_new_go = 0
        x= to_x
        y= to_y
        z= to_z
        self.publish_offboard_control_heartbeat_signal(True)
        self.distance_target = math.sqrt(math.pow(x-self.vehicle_odom.x,2)+math.pow(y-self.vehicle_odom.y,2)+math.pow((z-self.vehicle_odom.z),2))
        
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if ((self.distance_target < self.correction_range) | (self.setpoint_mode)):
                self.setpoint_mode = True
                self.publish_offboard_control_heartbeat_signal(False)
                self.stable_depart_publish(float(x), float(y), float(z), velo, yaw)
            else:
                self.publish_offboard_control_heartbeat_signal(False)
                self.publish_velocity_setpoint(float(x), float(y), float(z), velo, yaw)

        if (self.distance_target < self.waypoint_range) & (self.waypoint_count != (self.waypoint_num-1)):
            self.setpoint_mode = True
            self.wait_in_waypoint += 1
            if (self.wait_in_waypoint == 10):
                self.previous_waypoint[0] = self.vehicle_odom.x
                self.previous_waypoint[1] = self.vehicle_odom.y
                self.previous_waypoint[2] = self.vehicle_odom.z

                self.get_logger().info(f"{[to_x, to_y, to_z]}, departed !! ")

                self.is_new_go = 1
                self.is_departed = 1
                self.wait_in_waypoint = 0
                self.setpoint_mode = False

        if (self.offboard_setpoint_counter % 45 == 0):  #timer_callback 주기 1/15라고 가정, 3초마다 보정 
            #self.get_logger().info(f"** path correction with odom **")
            self.compensation_path_with_odom()

    def circle_path_publish(self, t_x: float, t_y: float, t_z: float, w:float, radius):
        msg = TrajectorySetpoint() 
        pi = math.pi
        diff_x = self.vehicle_odom.x - t_x
        diff_y = self.vehicle_odom.y - t_y
        if(math.sqrt(pow(diff_x,2)+pow(diff_y,2)) > (radius+1)):
            if(self.is_go_to_center==0):
                #self.get_logger().info(f" going to central point of circle ")
                self.is_go_to_center = 1
            self.goto_waypoint(t_x, t_y, t_z, self.waypoint_velocity[self.waypoint_count], 1)
            return
        self.publish_offboard_control_heartbeat_signal(True)
        if(self.initial_theta ==-1):
            self.get_logger().info(f" {[t_x, t_y, t_z]} circle path with r{radius} ")
            self.theta = math.atan2(diff_y, diff_x)
          
            self.initial_theta = self.theta
        msg.x = float(t_x+radius * np.cos(self.theta))
        msg.y = float(t_y+radius * np.sin(self.theta))
        msg.z = float(t_z)
        msg.yaw = float(self.theta + w * self.dt + math.pi) # (90 degree)
        self.theta = self.theta + w * self.dt
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def mission_check(self):
        if (self.waypoint_count == 2 or self.waypoint_count == 3 or self.waypoint_count==5):
            self.is_mission_ladder += 1

    def mission_ladder(self):

        if(self.is_mission_started == 1):
            se1 = SE(self.real_obstacle_list)
            self.get_logger().info(f" {[self.waypoint_list[self.waypoint_count][0], self.waypoint_list[self.waypoint_count][1], self.waypoint_list[self.waypoint_count][2]]}로 가기 위한 SE 알고리즘 경로를 생성합니다. ")
            self.waypoint_for_ladder = se1.make_final_path(self.waypoint_list[self.waypoint_count-1][0], self.waypoint_list[self.waypoint_count-1][1], self.waypoint_list[self.waypoint_count][0], self.waypoint_list[self.waypoint_count][1])
            for i in self.waypoint_for_ladder:
                i.append(self.waypoint_list[self.waypoint_count][2])
            self.is_mission_started = 0


        self.goto_waypoint(self.waypoint_for_ladder[self.ladder_mission_count][0], self.waypoint_for_ladder[self.ladder_mission_count][1], self.waypoint_for_ladder[self.ladder_mission_count][2], 
                           self.waypoint_velocity[self.waypoint_count], 1)

        if(self.is_departed == 1):
            self.ladder_mission_count += 1
            self.is_departed = 0

        if(self.ladder_mission_count == len(self.waypoint_for_ladder)):
            self.is_mission_started = 0
            self.ladder_mission_count = 0
            self.is_mission_ladder_finished = 1
    
    def make_points_for_contest(self, waypoint_1, waypoint_2, waypoint_3): 
        distance_w1_w2 = math.sqrt(pow(waypoint_1[0]-waypoint_2[0],2)+pow(waypoint_1[1]-waypoint_2[1],2)+pow(waypoint_1[2]-waypoint_2[2], 2))
        distance_w2_w3 = math.sqrt(pow(waypoint_2[0]-waypoint_3[0],2)+pow(waypoint_2[1]-waypoint_3[1],2)+pow(waypoint_2[2]-waypoint_3[2], 2))
        mission_point_1 = [waypoint_2[0]-(8.5*((waypoint_2[0]-waypoint_1[0])/distance_w1_w2)), waypoint_2[1]-(8.5*((waypoint_2[1]-waypoint_1[1])/distance_w1_w2)), waypoint_1[2]]
        mission_point_2 = [waypoint_2[0]+(8.5*((waypoint_3[0]-waypoint_2[0])/distance_w2_w3)), waypoint_2[1]+(8.5*((waypoint_3[1]-waypoint_2[1])/distance_w2_w3)), waypoint_2[2]]
        mission_point_3 = self.calculate_return_point(waypoint_2[0], waypoint_3[0], waypoint_2[1], waypoint_3[1])
        mission_point_3.append(waypoint_1[2])
        self.waypoint_list = [waypoint_1, mission_point_1, waypoint_2, mission_point_2, waypoint_3, mission_point_3, waypoint_1, [waypoint_1[0], waypoint_1[1], 0]]
        

    def calculate_return_point(self, a, b, c, d): #a->x2, b->x3, c->y2, d->y3 
        x = (a**3-2*(a**2)*b + a*(b**2+(c-d)**2)-15*math.sqrt(((a-b)**2) *(a**2-2*a*b+b**2+(c-d)**2)))/(a**2-2*a*b+b**2+(c-d)**2)
        y = ((a**3)*c-3*(a**2)*b*c-(b**3)*c+a*c*(3*(b**2)+(c-d)**2)-15*math.sqrt(((a-b)**2)*(a**2-2*a*b+b**2+(c-d)**2))*(c-d)-b*c*(c-d)**2)/((a-b)*(a**2-2*a*b+b**2+(c-d)**2))
        x2 = (a**3-2*(a**2)*b + a*(b**2+(c-d)**2)+15*math.sqrt(((a-b)**2) *(a**2-2*a*b+b**2+(c-d)**2)))/(a**2-2*a*b+b**2+(c-d)**2)
        y2 = ((a**3)*c-3*(a**2)*b*c-(b**3)*c+a*c*(3*(b**2)+(c-d)**2)+15*math.sqrt(((a-b)**2)*(a**2-2*a*b+b**2+(c-d)**2))*(c-d)-b*c*(c-d)**2)/((a-b)*(a**2-2*a*b+b**2+(c-d)**2))
    
        first_distance = pow(x-self.waypoint_list[0][0], 2)+pow(y-self.waypoint_list[0][1],2)
        second_distance = pow(x2-self.waypoint_list[0][0], 2)+pow(y2-self.waypoint_list[0][1],2)
        if(first_distance<second_distance):
            return [x,y]
        else:
            return [x2, y2]
        
    def mission_delivery(self):
        pass
    
    def ladder_detect_flight(self):
        self.circle_path_publish(self.waypoint_contest[1][0], self.waypoint_contest[1][1], self.waypoint_contest[1][2], 0.07, 8.5)
        if(self.theta-self.initial_theta > math.pi * 2):
            self.circle_path =0
            self.is_go_to_center = 0
            self.is_ladder_detected = 1
        

    def timer_callback(self) -> None:
        
        #self.get_logger().info(f"local position {[self.vehicle_odom.x, self.vehicle_odom.y, self.vehicle_odom.z]}")
        """Callback function for the timer."""
        x = float(self.waypoint_list[self.waypoint_count][0])
        y = float(self.waypoint_list[self.waypoint_count][1])
        z = float(self.waypoint_list[self.waypoint_count][2])
        v = float(self.waypoint_velocity[self.waypoint_count])
        yaw = float(self.waypoint_yaw[self.waypoint_count])
        
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            self.make_points_for_contest(self.waypoint_contest[0], self.waypoint_contest[1], self.waypoint_contest[2])
        
        if(self.is_mission_ladder == 1 and self.waypoint_count == 2 and self.is_mission_ladder_finished == 0):
            if(self.wait_time == 0):
                self.wait_time = self.offboard_setpoint_counter

            if(self.offboard_setpoint_counter-self.wait_time < 10 and self.is_ladder_detected == 0):
                self.publish_offboard_control_heartbeat_signal(False)
                self.stable_depart_publish(self.waypoint_list[self.waypoint_count-1][0], self.waypoint_list[self.waypoint_count-1][1], self.waypoint_list[self.waypoint_count-1][2], self.waypoint_velocity[self.waypoint_count-1], 0)
                if (self.offboard_setpoint_counter == 9):
                    self.wait_time = -1

            elif(self.is_ladder_detected == 0):
                self.ladder_detect_flight()
                if(self.is_ladder_detected == 1):
                    self.wait_time = self.offboard_setpoint_counter
            elif(self.is_ladder_detected == 1 and self.wait_time != -2):
                self.publish_offboard_control_heartbeat_signal(False)
                self.stable_depart_publish(self.waypoint_list[self.waypoint_count-1][0], self.waypoint_list[self.waypoint_count-1][1], self.waypoint_list[self.waypoint_count-1][2], self.waypoint_velocity[self.waypoint_count-1], 0)
                if(self.offboard_setpoint_counter - self.wait_time ==10):
                    self.wait_time = -2
            else:
                self.mission_ladder()
        
        elif(self.is_mission_ladder == 2 and self.waypoint_count == 3 and self.is_mission_ladder_finished == 0):
            self.mission_ladder()
        
        #elif(self.is_mission_ladder == 3 and self.waypoint_count == 5 and self.is_mission_ladder_finished == 0):
            #self.mission_ladder()

        elif(self.is_mission_delivery == 1):
            pass
        
        #elif(self.waypoint_count == 3 and self.circle_path == 1): # 몇 번째 waypoint를 중심으로 돌고 싶은지
        #    self.circle_path_publish(x, y, z, 0.2, 3)
        #    if (self.theta-self.initial_theta > math.pi *2 ): # 한바퀴 돌기
        #        self.circle_path = 0
        #        self.waypoint_count += 1e
        #        self.is_go_to_center = 0
        else:
            self.goto_waypoint(x, y, z, v, yaw)
       
        if(self.is_departed == 1 and (self.waypoint_count < self.waypoint_num)): # next waypoint update
            self.waypoint_count +=1
            self.is_mission_ladder_finished = 0
            self.is_mission_started = 1
            self.is_departed = 0
            self.mission_check()
        
        self.offboard_setpoint_counter += 1
       

       
        
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