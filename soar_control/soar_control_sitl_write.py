#!/usr/bin/env python3
# -* coding: utf-8 -*-
############################################################################
#   version 230724 0304
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
#from detect_object import detect_ladder, detect_balcony, detect_crossbow
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleOdometry
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point
from sklearn.cluster import KMeans

import math
import numpy as np

def plus_radian(w, plus_w):
    w += plus_w
    if(w>2*math.pi):
        w-=2*math.pi
    if(w<0):
        w+=2*math.pi
    return w 

#PX4' quaternion yaw f
def euler_from_quaternion(w, x, y, z): 
    t0 = +2 * (w*x+y*z)
    t1 = +1 - 2*(x*x+y*y)
    t3 = +2*(w*z+x*y)
    t4 = +1-2*(y*y+z*z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def make_orth_points(x_1, y_1, x_2, y_2, r):
    t_x1 = (x_1+x_2)/2 + math.cos(math.atan2(-x_2+x_1,y_2-y_1)) * r
    t_x2 = (x_1+x_2)/2 - math.cos(math.atan2(-x_2+x_1,y_2-y_1)) * r
    t_y1 =  (y_1+y_2)/2 + math.sin(math.atan2(-x_2+x_1,y_2-y_1)) * r
    t_y2 = (y_1+y_2)/2 - math.sin(math.atan2(-x_2+x_1,y_2-y_1)) * r
    result_list = [t_x1, t_y1, t_x2, t_y2]
    return result_list

def arrange_shortest_point(now_x, now_y, points):
    d1 = math.sqrt(pow(now_x-points[0], 2)+pow(now_y-points[1],2))
    d2 = math.sqrt(pow(now_x-points[2], 2)+pow(now_y-points[3],2))
    if(d2<d1):
        temp_x = points[0]
        temp_y = points[1]
        points[0] = points[2]
        points[1] = points[3]
        points[2] = temp_x
        points[3] = temp_y
    return points
        


class OffboardControl(Node):
    
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('SOAR_HAS_CONTROL')

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

        self.waypoint_contest = [[-1,-1,-16], [52.25, 100.65, -16], [84, 182, -16]] 
        self.waypoint_list = [[0,0,-2], [0,0,-2], [0,0,-2], [0,0,-2], [0,0,-2], [0,0,-2], [0, 0, -2], [0, 0, -2], [0, 0, -2], [0, 0, -2], [0, 0, 0]] 
        #                      way1     mission1     way2   mission2    way3     way3      mission2      way2       mission1      way1      landing
        self.waypoint_velocity = [2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2]
   
        self.waypoint_yaw = [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0]
        self.waypoint_num = len(self.waypoint_list)
        self.waypoint_count = 0
        self.waypoint_range = 0.4 
        self.correction_range = 2.5
        self.previous_waypoint = [0,0,0]
        self.wait_in_waypoint = 0
        self.previous_yaw = 0
        self.distance_target = 0
        self.is_go_to_center = 0 
        self.stable_counter = 0 
        self.stable_odom = [0, 0, 0]
        self.is_go_to_circle_point = 0
        self.stayed_finished = 0

        self.wait_time = 0
    
        self.is_new_go = 0 
        self.is_departed = 0 

        self.now_yaw = 0
        self.change_yaw = -1
        self.is_yaw_arranged = 0
        
        
        #variables for ladder
        self.is_mission_ladder = 0
        self.is_mission_delivery = 0
        self.is_mission_started = 0
        self.ladder_mission_count = 0
        self.is_ladder_mission_finished = 0
        self.is_ladder_detected = 0

        self.real_ladder_list = [[0, 0, 1.5], [0, 0, 1.5]]
        self.waypoint_for_ladder = []
    
        self.theta = 0
        self.initial_theta = -1
        self.initial_theta2 = -1 
        self.circle_path = 1
        self.sub_positions = []

        self.confirmed_ladder_num = 0

        self.ladder_list_guess = [[0, 0 ,1.5], [0,0,1.5]]
        self.point_list = []
        self.callback_flag = 0 



        self.is_mission_delivery = 0
        self.is_crossbow_detected = 0
        self.crossbow_location = [0, 0, -8] 
        self.crossbow_location_confirmed = [0, 0, -8] 

        self.is_delivery_started = 0
        self.theta_yaw = 0
        self.crossbow_showed_list = [] 
        self.crossbow_start_point = [0, 0, -7] 
        self.is_delivery_going = 0
        self.pizza_closed_point_distance = 0
        self.wait_in_deliverypoint = 0

        self.confirmed_balcony_location = [0, 0, -7]
        self.confirmed_balcony_num = 0
        self.confirmed_crossbow_num = 0      

        self.balcony_confirmed_x = 0
        self.balcony_confirmed_y = 0
        self.balcony_list = []
        self.cross_location = [0, 0, -8]
        self.crossbow_yaw_list = []
        self.crossbow_subscribing = 0
        self.previous_crossbow_msg = [0,0,0]

        # Create a timer to publish control commands
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback)


        # variables for emergency
        self.emergency_ladder_location = [[3.6, 48, 1.5], [-1.3, 52, 1.5]]
        self.emergency_balcony_location = [18.0475, 162.25, -8]
        self.emergency_crossbow_started_location = [18.87, 163.64, -8]
        self.emergency_crossbow_location = [17.98, 160.255, -8]
    def detect_ladder(self): 
        qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
        )
        grid_cells_subscriber = self.create_subscription(
            GridCells, '/map_talker/ladder', self.PCL_callback, 10)
        
        return self.ladder_list_guess
        
    def PCL_callback(self, msg):
        '''
        self.callback_flag = self.offboard_setpoint_counter
        if(self.callback_flag % 100 != 0 and self.callback_flag !=self.offboard_setpoint_counter):
          return
        self.callback_flag += 1
        #print(self.callback_flag)
        ''' 
        '''
        for i in msg.cells:
            distance = math.sqrt(pow(i.x*0.2, 2)+pow(i.y*0.2,2))
            if(distance>0 and distance < 14):
                self.point_list.append([i.x, i.y])
        if(len(self.point_list)<2):
            return
        '''
        '''
        k_means = KMeans(init="k-means++", n_clusters=2, n_init=10)
        k_means.fit(self.point_list)
        k_means_cluster_centers = k_means.cluster_centers_
        self.ladder_list_guess = k_means_cluster_centers
        '''
        if(len(msg.cells)==2):
            self.ladder_list_guess[0][0] = msg.cells[0].x
            self.ladder_list_guess[0][1] = msg.cells[0].y
            self.ladder_list_guess[1][0] = msg.cells[1].x
            self.ladder_list_guess[1][1] = msg.cells[1].y


    def detect_balcony(self):
        qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
        )
        grid_cells_subscriber2 = self.create_subscription(
            GridCells, '/map_talker/gridcells', self.PCL_callback2, 10)

        return [self.balcony_confirmed_x, self.balcony_confirmed_y]
    
    def PCL_callback2(self, msg):
        if(self.offboard_setpoint_counter % 4 !=0):
            return
        if(len(msg.cells)!=1):
 
            return
        self.balcony_confirmed_x = msg.cells[0].x
        self.balcony_confirmed_y = msg.cells[0].y
    
    def detect_crossbow(self):
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        #self.cross_location[0] = 0
        #self.cross_location[1] = 0
        self.cross_location_subscriber = self.create_subscription(
            Point, 's2s_result', self.cross_callback, 1)
    


    def vehicle_odom_callback(self, vehicle_odom):
        self.vehicle_odom = vehicle_odom
        self.now_yaw = euler_from_quaternion(self.vehicle_odom.q[0], self.vehicle_odom.q[1], self.vehicle_odom.q[2], self.vehicle_odom.q[3])
        
    def cross_callback(self, msg):
        if(msg.z<-9 or msg.z>-6.5):
            return
        distance_from_way3 = math.sqrt(math.pow(self.waypoint_contest[2][0]-msg.x,2)+math.pow(self.waypoint_contest[2][1]-msg.y,2))
        if(distance_from_way3>10):
            return        
        self.crossbow_location[0] = msg.x
        self.crossbow_location[1] = msg.y
        self.crossbow_location[2] = msg.z
        if(self.previous_crossbow_msg[0]==msg.x and self.previous_crossbow_msg[1]==msg.y):
            return

        if(self.crossbow_location[0]!=0):

            yaw = math.atan2(self.vehicle_odom.y-self.confirmed_balcony_location[1], self.vehicle_odom.x-self.confirmed_balcony_location[0])
            if(yaw<0):
                yaw += (math.pi*2)
            print(yaw)
            self.crossbow_yaw_list.append(yaw)
            self.previous_crossbow_msg[0] = msg.x
            self.previous_crossbow_msg[1] = msg.y
            self.previous_crossbow_msg[2] = msg.z        


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

    def ladder_detect_on(self): 
        guess_ladder = self.detect_ladder()
        if(len(guess_ladder) != 2): 
            return
        ladder1_x = guess_ladder[0][0]
        ladder1_y = guess_ladder[0][1]
        ladder2_x = guess_ladder[1][0]
        ladder2_y = guess_ladder[1][1]
        distance = math.sqrt(math.pow(ladder1_x-ladder2_x,2)+math.pow(ladder1_y-ladder2_y,2))
        distance_waypoint = math.sqrt(math.pow((ladder1_x+ladder2_x)/2 - self.waypoint_contest[1][0], 2)+math.pow((ladder1_y+ladder2_y)/2 -self.waypoint_contest[1][1], 2))
        
        d1 = math.sqrt(math.pow(self.real_ladder_list[0][0]-ladder1_x,2)+math.pow(self.real_ladder_list[0][1]-ladder1_y,2))
        d2 = math.sqrt(math.pow(self.real_ladder_list[1][0]-ladder1_x,2)+math.pow(self.real_ladder_list[1][1]-ladder1_y,2))
        if(d1>d2):
            temp_x = ladder1_x
            temp_y = ladder1_y
            ladder1_x = ladder2_x
            ladder1_y = ladder2_y
            ladder2_x = temp_x
            ladder2_y = temp_y             

        if(distance<9 and distance>5 and distance_waypoint<2):
            if(self.confirmed_ladder_num==0):
                self.real_ladder_list[0][0] = ladder1_x
                self.real_ladder_list[0][1] = ladder1_y
                self.real_ladder_list[1][0] = ladder2_x
                self.real_ladder_list[1][1] = ladder2_y 
                self.confirmed_ladder_num+=1
            else:
                self.real_ladder_list[0][0] = (self.real_ladder_list[0][0]) * (self.confirmed_ladder_num/(self.confirmed_ladder_num+1)) + ladder1_x/(self.confirmed_ladder_num+1)
                self.real_ladder_list[0][1] = (self.real_ladder_list[0][1]) * (self.confirmed_ladder_num/(self.confirmed_ladder_num+1)) + ladder1_y/(self.confirmed_ladder_num+1)
                self.real_ladder_list[1][0] = (self.real_ladder_list[1][0]) * (self.confirmed_ladder_num/(self.confirmed_ladder_num+1)) + ladder2_x/(self.confirmed_ladder_num+1)
                self.real_ladder_list[1][1] = (self.real_ladder_list[1][1]) * (self.confirmed_ladder_num/(self.confirmed_ladder_num+1)) + ladder2_y/(self.confirmed_ladder_num+1)
                self.confirmed_ladder_num += 1
                print("ladder 위치 보정됨 : " + str(self.real_ladder_list))
           
    

    def balcony_detect_on(self):
        guess_balcony = self.detect_balcony()
        if(len(guess_balcony)==0):
            return 
        if(guess_balcony[0]==0 and guess_balcony[1]==0):
            return

        if(self.confirmed_balcony_num ==0): 
            self.confirmed_balcony_location[0] = guess_balcony[0]
            self.confirmed_balcony_location[1] = guess_balcony[1]
            self.confirmed_balcony_num += 1 
        else:
            self.confirmed_balcony_location[0] = (self.confirmed_balcony_location[0]) * (self.confirmed_balcony_num/(self.confirmed_balcony_num+1)) + guess_balcony[0]/(self.confirmed_balcony_num+1)
            self.confirmed_balcony_location[1] = (self.confirmed_balcony_location[1]) * (self.confirmed_balcony_num/(self.confirmed_balcony_num+1)) + guess_balcony[1]/(self.confirmed_balcony_num+1)
            self.confirmed_balcony_num += 1 
            print("balcony 위치 보정됨 : " + str(self.confirmed_balcony_location))
        #print(2)
        #print(self.confirmed_balcony_location[0])
        #print(self.confirmed_balcony_location[1])
    def cross_detect_on(self):
        if(self.is_crossbow_detected == 0): 
            
            self.detect_crossbow()
        

    
        if(self.is_crossbow_detected == 1):
            #guess_crossbow = self.detect_crossbow()
            self.detect_crossbow()
            if(self.crossbow_location[0]==0 and self.crossbow_location[1]==0): 
                return 0
            if(self.confirmed_crossbow_num == 0):
                self.crossbow_location_confirmed[0] = self.crossbow_location[0]
                self.crossbow_location_confirmed[1] = self.crossbow_location[1]
                self.crossbow_location_confirmed[2] = self.crossbow_location[2]
                self.confirmed_crossbow_num +=1
            else:
                cross_distance = math.sqrt(math.pow(self.vehicle_odom.x - self.crossbow_location_confirmed[0],2)+math.pow(self.vehicle_odom.y-self.crossbow_location_confirmed[1],2))
                if(cross_distance<4):
                    self.crossbow_location_confirmed[0] = (self.crossbow_location_confirmed[0]) * (self.confirmed_crossbow_num/(self.confirmed_crossbow_num+1)) + self.crossbow_location[0]/(self.confirmed_crossbow_num+1)
                    self.crossbow_location_confirmed[1] = (self.crossbow_location_confirmed[1]) * (self.confirmed_crossbow_num/(self.confirmed_crossbow_num+1)) + self.crossbow_location[1]/(self.confirmed_crossbow_num+1)
                    self.crossbow_location_confirmed[2] = (self.crossbow_location_confirmed[2]) * (self.confirmed_crossbow_num/(self.confirmed_crossbow_num+1)) + self.crossbow_location[2]/(self.confirmed_crossbow_num+1)
                    self.confirmed_crossbow_num += 1 
            return 1
        return 1

                


    def publish_offboard_control_heartbeat_signal(self, is_p): 
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        if is_p: ## if is_p == true: 
   
            msg.velocity = False
            msg.position = True 

        else: ## if is_p == false:
            msg.velocity = True
            msg.position = False 
        
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_yaw_with_hovering(self, x:float, y:float, z:float, v:float, yaw:float): 
        msg = TrajectorySetpoint()
        self.publish_offboard_control_heartbeat_signal(True) 
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.yaw = float(yaw)

        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.trajectory_setpoint_publisher.publish(msg)



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
        self.publish_offboard_control_heartbeat_signal(False)
        pi = math.pi
        self.previous_waypoint[0] = self.vehicle_odom.x
        self.previous_waypoint[1] = self.vehicle_odom.y
        self.previous_waypoint[2] = self.vehicle_odom.z
        self.stable_counter+=1 
        xy_distance = math.sqrt(math.pow(t_x-self.previous_waypoint[0],2)+math.pow(t_y-self.previous_waypoint[1], 2))

        self.stable_odom[0] += self.vehicle_odom.x
        self.stable_odom[1] += self.vehicle_odom.y
        self.stable_odom[2] += self.vehicle_odom.z
        
        if(self.stable_counter % 3 == 0):
            self.previous_waypoint[0] = self.stable_odom[0]/3
            self.previous_waypoint[1] = self.stable_odom[1]/3
            self.previous_waypoint[2] = self.stable_odom[2]/3
            self.stable_odom = [0, 0, 0]

        if(self.distance_target < self.waypoint_range): 
            if(v < 0.65): 
                self.publish_velocity_setpoint(t_x, t_y, t_z, v*(self.distance_target/self.correction_range), 0) 
                return
            else:
                self.publish_velocity_setpoint(t_x, t_y, t_z, v*(pow(self.distance_target,2)/pow(self.correction_range,2)),0) 
                return
        else:
            if(v < 0.65):
                if(xy_distance<0.5):
                    self.publish_velocity_setpoint(t_x, t_y, t_z, v*(math.sqrt(self.distance_target)/math.sqrt(self.correction_range)), 0)
                else:
                    self.publish_velocity_setpoint(t_x, t_y, t_z, v*(math.sqrt(self.distance_target)/math.sqrt(self.correction_range)), 1)
                return 
            else:
                if(xy_distance<0.5):
                    self.publish_velocity_setpoint(t_x, t_y, t_z, v*(math.sqrt(self.distance_target)/math.sqrt(self.correction_range)), 0)
                else:
                    self.publish_velocity_setpoint(t_x, t_y, t_z, v*(math.sqrt(self.distance_target)/math.sqrt(self.correction_range)), 1)
     
    def publish_velocity_setpoint(self, t_x: float, t_y: float, t_z:float, v:float, yaw:float): 
        plus_yaw = 0.07

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
        
        t_yaw = math.atan2(diff_y, diff_x)
        if(t_yaw<0):
            t_yaw += math.pi*2
        if(self.now_yaw<0):
            self.now_yaw += math.pi*2

        if(yaw!=0):
            if(t_yaw-self.now_yaw>=4*plus_yaw and t_yaw-self.now_yaw<pi):
                if(self.change_yaw==-1):
                    self.change_yaw = self.now_yaw
                msg.vx = float(0)
                msg.vy = float(0)
                msg.vz = float(0)
                self.change_yaw = plus_radian(self.change_yaw, plus_yaw) 
            elif(t_yaw-self.now_yaw>=pi and t_yaw-self.now_yaw<=2*pi-4*plus_yaw):
                if(self.change_yaw == -1):
                    self.change_yaw = self.now_yaw
                msg.vx = float(0)
                msg.vy = float(0)
                msg.vz = float(0)
                self.change_yaw = plus_radian(self.change_yaw, -plus_yaw)
            elif(self.now_yaw-t_yaw>4*plus_yaw and self.now_yaw-t_yaw<=pi):
                if(self.change_yaw == -1):
                    self.change_yaw = self.now_yaw
                msg.vx = float(0)
                msg.vy = float(0)
                msg.vz = float(0)
                self.change_yaw = plus_radian(self.change_yaw, -plus_yaw)
            elif(self.now_yaw-t_yaw>pi and self.now_yaw-t_yaw<=2*pi-4*plus_yaw):
                if(self.change_yaw == -1):
                    self.change_yaw = self.now_yaw
                msg.vx = float(0)
                msg.vy = float(0)
                msg.vz = float(0)
                self.change_yaw = plus_radian(self.change_yaw, plus_yaw)
            else:
                self.change_yaw = t_yaw
                
            msg.yaw = float(self.change_yaw) 
            if(t_yaw == self.change_yaw):
                self.change_yaw = -1
            msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
            self.trajectory_setpoint_publisher.publish(msg)
        else:
            msg.yaw = float(self.previous_yaw)
            msg.timestamp = int(self.get_clock().now().nanoseconds/1000) 
            self.trajectory_setpoint_publisher.publish(msg)
        self.previous_yaw = msg.yaw


    def compensation_path_with_odom(self): 
        self.previous_waypoint[0] = self.vehicle_odom.x
        self.previous_waypoint[1] = self.vehicle_odom.y
        self.previous_waypoint[2] = self.vehicle_odom.z

    def publish_vehicle_command(self, command, **params) -> None:
 
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


        x = to_x  
        y = to_y
        z = to_z
                                                               
        self.publish_offboard_control_heartbeat_signal(False) 
        self.distance_target = math.sqrt(math.pow(x-self.vehicle_odom.x,2) + math.pow(y-self.vehicle_odom.y,2) + math.pow((z-self.vehicle_odom.z),2)) 
        
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if (self.distance_target < self.correction_range):
                self.stable_depart_publish(float(x), float(y), float(z), velo, yaw)
            else:
                self.publish_velocity_setpoint(float(x), float(y), float(z), velo, yaw)
            
        if (self.distance_target < self.waypoint_range) & (self.waypoint_count != (self.waypoint_num-1)): 

            self.wait_in_waypoint += 1
            if (self.wait_in_waypoint == 10): 
                self.previous_waypoint[0] = self.vehicle_odom.x
                self.previous_waypoint[1] = self.vehicle_odom.y
                self.previous_waypoint[2] = self.vehicle_odom.z

                self.get_logger().info(f"{[to_x, to_y, to_z]}, departed !! ")

                self.is_new_go = 1
                self.is_departed = 1
                self.wait_in_waypoint = 0

        if (self.offboard_setpoint_counter % 45 == 0): 
            self.compensation_path_with_odom()

    def circle_path_publish(self, t_x: float, t_y: float, t_z: float, w:float, radius):
        msg = TrajectorySetpoint() 
        pi = math.pi
        diff_x = self.vehicle_odom.x - t_x
        diff_y = self.vehicle_odom.y - t_y
        if(self.is_go_to_center==0):
            self.initial_theta2 = math.atan2(diff_y, diff_x)
            self.is_go_to_center = 1
        
        positive_now_yaw = self.now_yaw
        if(positive_now_yaw<0):
            positive_now_yaw += 2*math.pi
        positive_initial_theta = math.atan2(diff_y, diff_x)
        positive_initial_theta += math.pi
    
        if(self.is_yaw_arranged==0):
            
            self.goto_waypoint(t_x+math.cos(self.initial_theta2) * radius, t_y+math.sin(self.initial_theta2)*radius, t_z, self.waypoint_velocity[self.waypoint_count], 1)
            if(self.is_departed==1):
                self.is_departed=0
                self.is_yaw_arranged=1
            return
            
        elif abs(positive_now_yaw-positive_initial_theta)>0.35 and abs(positive_now_yaw-positive_initial_theta)<(math.pi*2-0.35):
            self.publish_velocity_setpoint(t_x, t_y, t_z, 0.1, 1)
            return      
        
        else:
            self.publish_offboard_control_heartbeat_signal(True) 
            if(self.initial_theta == -1): 
                self.get_logger().info(f" {[t_x, t_y, t_z]} circle path with r = {radius} m")
                self.theta = math.atan2(diff_y, diff_x)
                self.initial_theta = self.theta
            msg.x = float(t_x+radius * np.cos(self.theta))
            msg.y = float(t_y+radius * np.sin(self.theta))
            msg.z = float(t_z)
            msg.yaw = float(self.theta + w * self.dt + math.pi) # (90 degree)
            self.theta = self.theta + w * self.dt
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_setpoint_publisher.publish(msg)
            self.is_yaw_arranged = -1
    
    def mission_check(self):
        if (self.waypoint_count == 2 or self.waypoint_count == 3 or self.waypoint_count == 7 or self.waypoint_count == 8): 
 
            self.is_mission_ladder += 1
        if (self.waypoint_count == 5 and self.is_mission_delivery == 0): 
            self.get_logger().info(" departed at wpt 3 and delivering ")
            self.is_mission_delivery = 1 


    def mission_ladder(self):

        if(self.is_mission_started == 1): 
            se1 = SE(self.real_ladder_list) 
            self.sub_positions = make_orth_points(self.real_ladder_list[0][0], self.real_ladder_list[0][1], self.real_ladder_list[1][0], self.real_ladder_list[1][1], 2.5)
            arrange_shortest_point(self.waypoint_list[self.waypoint_count-1][0], self.waypoint_list[self.waypoint_count-1][1], self.sub_positions)
            self.get_logger().info(f" {[self.waypoint_list[self.waypoint_count][0], self.waypoint_list[self.waypoint_count][1], self.waypoint_list[self.waypoint_count][2]]}로 가기 위한 SE 알고리즘 경로를 생성합니다. ")
            
            if(self.waypoint_count == 2 or self.waypoint_count==7):
                self.waypoint_for_ladder = se1.make_final_path(self.waypoint_list[self.waypoint_count-1][0], self.waypoint_list[self.waypoint_count-1][1], self.sub_positions[0], self.sub_positions[1])
                self.waypoint_for_ladder.append([self.sub_positions[0], self.sub_positions[1]])
            elif(self.waypoint_count == 3 or self.waypoint_count == 8):
                self.waypoint_for_ladder = se1.make_final_path(self.sub_positions[2], self.sub_positions[3], self.waypoint_list[self.waypoint_count][0], self.waypoint_list[self.waypoint_count][1])
                self.waypoint_for_ladder.insert(0, [self.sub_positions[2], self.sub_positions[3]])
            
            ## ^ make_final_path(self, now_x, now_y, target_x, target_y)
            for i in self.waypoint_for_ladder: 
                i.append(self.waypoint_list[self.waypoint_count][2]) 
            self.is_mission_started = 0 


        self.goto_waypoint(self.waypoint_for_ladder[self.ladder_mission_count][0], self.waypoint_for_ladder[self.ladder_mission_count][1], self.waypoint_for_ladder[self.ladder_mission_count][2], 
                           self.waypoint_velocity[self.waypoint_count], 1) ## 

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
        self.waypoint_list = [waypoint_1, mission_point_1, waypoint_2, mission_point_2, waypoint_3, waypoint_3, mission_point_2, waypoint_2, mission_point_1, waypoint_1,  [0, 0, -1]]
                            #   0                1            2             3               4            5               6              7             8                    9
        
    def mission_delivery(self):
        delivery_velocity = 0.5 
        if(self.is_delivery_going == 0):
            if(self.is_crossbow_detected == 0):
                self.cross_bow_detect()
                if(self.is_crossbow_detected == 1):
                    self.get_logger().info(f" crossbow detected, ready to go delivery ")
            else:
                self.goto_waypoint(self.crossbow_start_point[0], self.crossbow_start_point[1], -7, self.waypoint_velocity[self.waypoint_count], 1)
                if(self.is_departed == 1):
                    self.is_delivery_going = 1
                    self.wait_in_deliverypoint = self.offboard_setpoint_counter
                    self.is_departed = 0
        elif(self.is_delivery_going == 1):
            yaw_to_crossbow = math.atan2(self.crossbow_location_confirmed[1]-self.crossbow_start_point[1], self.crossbow_location_confirmed[0]-self.crossbow_start_point[0])
            self.publish_yaw_with_hovering(self.crossbow_start_point[0], self.crossbow_start_point[1], -7, self.waypoint_velocity[self.waypoint_count], yaw_to_crossbow)
            if(self.offboard_setpoint_counter - self.wait_in_deliverypoint > 20):
                self.get_logger().info(f" pizza is going " )
                self.is_delivery_going = 2
                self.wait_in_delivery_point = 0
        elif(self.is_delivery_going == 2):
            # to crossbow
            self.cross_detect_on() 

            if(self.crossbow_location_confirmed[0]==0 and self.crossbow_location_confirmed[1]==0):
                self.crossbow_location_confirmed = self.emergency_crossbow_location
                self.get_logger().info(f" fail to detect cross ")

            if(self.pizza_closed_point_distance==0):
                self.pizza_closed_point_distance = math.sqrt(pow(self.crossbow_location_confirmed[0]-self.crossbow_start_point[0],2)+pow(self.crossbow_location_confirmed[1]-self.crossbow_start_point[1],2))
            d = self.pizza_closed_point_distance

                
            self.goto_waypoint(self.crossbow_location_confirmed[0]+(3/d)*(self.crossbow_start_point[0]-self.crossbow_location_confirmed[0]), self.crossbow_location_confirmed[1]+(3/d)*(self.crossbow_start_point[1]-self.crossbow_location_confirmed[1]), self.crossbow_location_confirmed[2], delivery_velocity, 1)
            
            if(self.is_departed == 1):
                if(self.wait_in_delivery_point == 0):
                    self.wait_in_delivery_point = self.offboard_setpoint_counter
                if(self.offboard_setpoint_counter - self.wait_in_delivery_point == 40):
                    self.is_delivery_going = 3
                self.is_departed = 0
        elif(self.is_delivery_going == 3):
            self.get_logger().info(f" delivery finished ")
            self.is_mission_delivery = -1
    
    def cross_bow_detect(self):
        w = 0.025 ## radian
        if (self.is_delivery_started == 0): 
            #self.goto_waypoint(self.waypoint_contest[2][0], self.waypoint_contest[2][1], -7, 1, 0)
            self.goto_waypoint(self.waypoint_contest[2][0],self.waypoint_contest[2][1],-7,1,0) 
            if(self.is_departed == 1): 
                self.is_departed = 0
                self.is_delivery_started = -1

        elif (self.is_delivery_started < 0):
            if(self.is_delivery_started == -1): 
                self.theta_yaw = euler_from_quaternion(self.vehicle_odom.q[0], self.vehicle_odom.q[1], self.vehicle_odom.q[2], self.vehicle_odom.q[3]) 
            self.theta_yaw += w
            self.balcony_detect_on() 
  
            self.publish_yaw_with_hovering(self.waypoint_contest[2][0], self.waypoint_contest[2][1], -7, 2, self.theta_yaw) 
            self.is_delivery_started -= 1
            if(self.is_delivery_started < -(2*math.pi/w)): 
                if(self.confirmed_balcony_location[0]==0 and self.confirmed_balcony_location[1]==0):
                    self.get_logger().info(f" balcony detect failed. use emergency balcony location ")
                    self.confirmed_balcony_location = self.emergency_balcony_location[:]
                    self.is_delivery_started = 1
                else:
                    self.get_logger().info(f" balcony detected ") 
                    self.is_delivery_started = 1
      
        else :
            #print(1)
            self.circle_path_publish(self.confirmed_balcony_location[0], self.confirmed_balcony_location[1], -7.5, 0.1, 8.5) 
       
            self.cross_detect_on()
            if (self.theta-self.initial_theta > math.pi *2 ): 
                self.circle_path = 0
                self.is_go_to_center = 0
                self.is_go_to_circle_point = 0
                self.is_crossbow_detected = 1

                crossbow_showed_list_num = len(self.crossbow_showed_list)
                self.crossbow_start_point = [0, 0, 0]
                '''
                for i in self.crossbow_showed_list:
                    self.crossbow_start_point[0] += i[0]
                    self.crossbow_start_point[1] += i[1]
                    self.crossbow_start_point[2] += i[2]
                if(crossbow_showed_list_num != 0):
                    self.crossbow_start_point[0] = self.crossbow_start_point[0]/crossbow_showed_list_num
                    self.crossbow_start_point[1] = self.crossbow_start_point[1]/crossbow_showed_list_num
                    self.crossbow_start_point[2] = self.crossbow_start_point[2]/crossbow_showed_list_num 
                if(self.crossbow_start_point[0]==0 and self.crossbow_start_point[1]==0):
                    self.get_logger().info(f" fail to find proper location to align drone with crossbow. use emergency location ")
                    self.crossbow_start_point = self.emergency_crossbow_started_location
                else:
                    self.get_logger().info(f" cross detected ")
                '''
                total_yaw = 0
                for y in self.crossbow_yaw_list:
                    total_yaw += y
                if (len(self.crossbow_yaw_list)!=0):
                    total_yaw /= len(self.crossbow_yaw_list)
                    self.crossbow_start_point[0] = self.confirmed_balcony_location[0]+math.cos(total_yaw) * 8.5
                    self.crossbow_start_point[1] = self.confirmed_balcony_location[1]+math.sin(total_yaw) * 8.5
                    self.crossbow_start_point[2] = self.crossbow_location[2]
                else:
                    self.get_logger().info(f"fail to find proper location to align drone with crossbow. use emergency location ")
                    self.crossbow_start_point = self.emergency_crossbow_started_location 
                #print(self.crossbow_yaw_list)

    def ladder_detect_flight(self):
        self.circle_path_publish(self.waypoint_contest[1][0], self.waypoint_contest[1][1], self.waypoint_contest[1][2], 0.25, 8.5) 
    
        self.ladder_detect_on()
        if (self.theta-self.initial_theta > math.pi * 2):
            self.circle_path = 0
            self.is_go_to_center = 0
            self.is_ladder_detected = 1
            self.initial_theta = -1
            if(self.real_ladder_list[0][0]==0 and self.real_ladder_list[0][1]==0):
                self.get_logger().info(f" ladder detected failed. use emergency ladder location ")
                self.real_ladder_list = self.emergency_ladder_location
            else:
                self.get_logger().info(f" sucess to confirm ladder location ")
    
    def stay_in_moment(self, x, y, z, v, yaw):
        
        if(self.wait_time == 0):
            self.get_logger().info(f" wait for moment ..  ")
            self.wait_time = self.offboard_setpoint_counter
        
        if(self.offboard_setpoint_counter - self.wait_time < 10):
            self.publish_offboard_control_heartbeat_signal(False)
            self.stable_depart_publish(x, y, z, v, yaw)
            
            if(self.offboard_setpoint_counter-self.wait_time == 9):
                self.wait_time = 0
                self.stayed_finished = 1

    def waypoint_update(self):
        self.waypoint_count +=1
        self.is_mission_ladder_finished = 0 
        self.is_mission_started = 1 
        self.is_departed = 0
        self.mission_check()

    def timer_callback(self) -> None:
        if(self.offboard_setpoint_counter % 20 == 0 and self.offboard_setpoint_counter>21):
            (self.f).write('['+str(self.vehicle_odom.x)+", "+str(self.vehicle_odom.y)+", "+str(self.vehicle_odom.z)+'],'+"\n")
        
        #self.get_logger().info(f"local position {[self.vehicle_odom.x, self.vehicle_odom.y, self.vehicle_odom.z]}")
        """Callback function for the timer."""
        x = float(self.waypoint_list[self.waypoint_count][0])
        y = float(self.waypoint_list[self.waypoint_count][1])
        z = float(self.waypoint_list[self.waypoint_count][2])
        v = float(self.waypoint_velocity[self.waypoint_count])
        yaw = float(self.waypoint_yaw[self.waypoint_count])
        
        if self.offboard_setpoint_counter == 10: 
            self.engage_offboard_mode
            self.arm()
            self.make_points_for_contest(self.waypoint_contest[0], self.waypoint_contest[1], self.waypoint_contest[2]) 
            self.previous_yaw = self.now_yaw
        if(self.waypoint_count == 2 and self.is_mission_ladder_finished == 0): 
            if(self.stayed_finished == 0):
                self.stay_in_moment(self.waypoint_list[self.waypoint_count-1][0], self.waypoint_list[self.waypoint_count-1][1], self.waypoint_list[self.waypoint_count-1][2], self.waypoint_velocity[self.waypoint_count-1], 0)
            
            elif(self.is_ladder_detected == 0):
                self.ladder_detect_flight() 
                if(self.is_ladder_detected == 1):
                    self.stayed_finished = 0
            else:
                self.mission_ladder() 
        
        elif(self.waypoint_count == 3 and self.is_mission_ladder_finished == 0): 
            self.mission_ladder()

        elif(self.waypoint_count == 7 and self.is_mission_ladder_finished == 0):
            self.mission_ladder()
        
        elif(self.waypoint_count == 8 and self.is_mission_ladder_finished == 0): 
            self.mission_ladder()

        elif(self.is_mission_delivery == 1): 
            self.mission_delivery()
            self.is_departed = 0
        
        else:
            self.goto_waypoint(x, y, z, v, yaw) 
       
        if(self.is_departed == 1 and (self.waypoint_count < self.waypoint_num)): 
            self.waypoint_update()
            if(self.waypoint_count == self.waypoint_num):
                (self.f).close()

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
