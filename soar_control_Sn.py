#!/usr/bin/env python3
# -*- coding: utf-8 -*-
############################################################################
#   version 230708 2035
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

#PX4' quaternion yaw 
def euler_from_quaternion(w, x, y, z): 
    t0 = +2 * (w*x+y*z)
    t1 = +1 - 2*(x*x+y*y)
    t3 = +2*(w*z+x*y)
    t4 = +1-2*(y*y+z*z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z


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

        self.waypoint_contest = [[0,0,-2], [70, 30, -2], [140, 100, -2]] ##대회에서 주는 wpt 3개
        self.waypoint_list = [[0,0,-2], [0,0,-2], [0,0,-2], [0,0,-2], [0,0,-2], [0,0,-2], [0, 0, -2], [0, 0, -2], [0, 0, -2], [0, 0, -2], [0, 0, 0]] ## 코드 상 wpt들... ## 변환은 밑 함수에서 함
        #                      way1     mission1     way2   mission2    way3     way3      mission2      way2       mission1      way1      landing
        self.waypoint_velocity = [2, 5, 5, 5, 5, 5, 5, 5, 5, 5, 2]
        self.waypoint_yaw = [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0] ## yaw 가 1인 건 heading을 고정하는 것...
        self.waypoint_num = len(self.waypoint_list)
        self.waypoint_count = 0
        self.waypoint_range = 0.4 ## wpt 인정 기준 1m
        self.correction_range = 2.5
        self.previous_waypoint = [0,0,0]
        self.wait_in_waypoint = 0
        self.setpoint_mode = False ## setpoint_mode 가 true : position 제어 / false : velocity 제어
        self.previous_yaw = 0
        self.distance_target = 0
        self.is_go_to_center = 0 ## ?
        self.stable_counter = 0 ## ?
        self.stable_odom = [0, 0, 0]
        self.is_go_to_circle_point = 0

        self.wait_time = 0
    
        self.is_new_go = 0 ## 다음 wpt가 존재하는지
        self.is_departed = 0 ## wpt에 도달했는지 
        
        
        #variables for ladder
        self.is_mission_ladder = 0
        self.is_mission_delivery = 0
        self.is_mission_started = 0
        self.ladder_mission_count = 0
        self.is_ladder_mission_finished = 0
        self.is_ladder_detected = 0

        self.real_obstacle_list = [[72.5, 32.5, 1.5], [67.5, 27.5, 1.5]] ## wpt2 로부터 3.5m 떨어진 위치로 사다리 위치 설정한 임의의 값
        self.waypoint_for_ladder = [] ## ladder 미션 시/ SE Algorithm을 통해 생성되는 point 저장 list 
    
        self.theta = 0
        self.initial_theta = -1 ## theta 초기 설정값.. 실제 존재하지 않는 값으로 설정
        self.initial_theta2 = -1 
        self.circle_path = 1

        #variables for delivery
        self.is_mission_delivery = 0
        self.is_crossbow_detected = 0
        self.crossbow_location = [144, 105, -8] ## 드론의 배란다 원주 비행 시 처음 들어오는 + 위치 저장 ; NED 좌표계 상의 값이므로 변화 x ## 이게 필요하지는 않은 듯? junalee의 요청..
        self.crossbow_location_confirmed = [144, 105, -8] ## crossbow_start_position에서 3m 간격 위치까지 이동하며 crossbow_location을 3개의 평균값으로 보정?? ## topic 세 번 발행 때마다 들어온 위치 값의 평균
        self.balcony_location = [143,106, -7]
        self.is_delivery_started = 0
        self.theta_yaw = 0
        self.crossbow_showed_list = [] #이중 리스트
        self.crossbow_start_point = [149, 106, -7] ## ? ; 계산으로 얻는 것 아니었나? 어떻게 처음부터 알고 있을까 ##임의로 설정
        self.is_delivery_going = 0
        self.pizza_closed_point_distance = 0
        self.wait_in_deliverypoint = 0

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

    def publish_offboard_control_heartbeat_signal(self, is_p): ## ? ; 지금은 velocity 제어만 하는데 이 함수가 왜 필요할까? ## circle path 시 사용
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        if is_p: ## if is_p == true: 
            ## position 제어
            msg.velocity = False
            msg.position = True 

        else: ## if is_p == false:
            ## velocity 제어
            msg.velocity = True
            msg.position = False 
        
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_yaw_with_hovering(self, x:float, y:float, z:float, v:float, yaw:float): ## wpt 3에서 자전 시 사용
        msg = TrajectorySetpoint()
        self.publish_offboard_control_heartbeat_signal(True) ## ? : 호버링할 때 true 여야 하는 이유? ## 두 방법이 있다만.. 충분히 안정적일 것이라....함..
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.yaw = float(yaw)

        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.trajectory_setpoint_publisher.publish(msg)



    def publish_position_setpoint(self, t_x: float, t_y: float, t_z: float, v:float, yaw:float): ## ? : 안 쓰는 건디 지워도 되는 거 아님? ## 됨......
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


    def stable_depart_publish(self, t_x: float, t_y : float, t_z : float, v:float, yaw:float): ## 안정적으로 wpt 도달할 수 있도록 속도 제어 .. wpt_range 기준 v 함수 변경..
        self.publish_offboard_control_heartbeat_signal(False)
        pi = math.pi
        self.previous_waypoint[0] = self.vehicle_odom.x
        self.previous_waypoint[1] = self.vehicle_odom.y
        self.previous_waypoint[2] = self.vehicle_odom.z
        self.stable_counter+=1 ## 토픽 발행 될 때마다 1씩 증가
        
        ## 토픽 발생 시마다 xyz값 각각 저장
        self.stable_odom[0] += self.vehicle_odom.x
        self.stable_odom[1] += self.vehicle_odom.y
        self.stable_odom[2] += self.vehicle_odom.z
        
        if(self.stable_counter % 3 == 0): ## 토픽 세 번 발행될 때마다의 odom xyz 평균값을 previous_wpt에 저장. wpt 인정(토픽 20번동안 위치 동일) 중 튀는 값이 들어오는 것 방지.... ##이걸로 괜찮아지나? TODO 공수치 ㅋㅋ
            self.previous_waypoint[0] = self.stable_odom[0]/3
            self.previous_waypoint[1] = self.stable_odom[1]/3
            self.previous_waypoint[2] = self.stable_odom[2]/3
            self.stable_odom = [0, 0, 0]

        if(self.distance_target < self.waypoint_range): ## waypoint_range 안에 드론이 들어온다면:
            if(v < 0.65): ## ``range 안에서 v < 0.65 일 때: ##0.65는 실험적으로 설정한 값
                self.publish_velocity_setpoint(t_x, t_y, t_z, v*(self.distance_target/self.correction_range), 0) ## 선형적으로 속도 감소
                return
            else:
                self.publish_velocity_setpoint(t_x, t_y, t_z, v*(pow(self.distance_target,2)/pow(self.correction_range,2)),0) ## 이후 
                return
        else:
            if(v < 0.65):
                self.publish_velocity_setpoint(t_x, t_y, t_z, v*(math.sqrt(self.distance_target)/math.sqrt(self.correction_range)), 0)
                return 
            else:
                self.publish_velocity_setpoint(t_x, t_y, t_z, v*(self.distance_target/self.correction_range), 0) ##? 얘는 return을 안 하네

    def publish_velocity_setpoint(self, t_x: float, t_y: float, t_z:float, v:float, yaw:float): ## (t_x, t_y, t_z)에 v의 속도로 가도록
        pi = math.pi
        x = self.previous_waypoint[0]
        y = self.previous_waypoint[1]
        z = self.previous_waypoint[2]

        
        msg = TrajectorySetpoint() 
        msg.x = np.nan
        msg.y = np.nan
        msg.z = np.nan
        
        if(t_x==x and t_y == y and t_z==z): ## 분모나 분자에 0이 들어가는 상황 제외
            msg.vx = float(0)
            msg.vy = float(0)
            msg.vz = float(0)
        else:
            msg.vx = v * ((t_x-x)/(math.sqrt(math.pow(t_x-x,2)+math.pow(t_y-y,2)+math.pow(t_z-z,2))))
            msg.vy = v * ((t_y-y)/(math.sqrt(math.pow(t_x-x,2)+math.pow(t_y-y,2)+math.pow(t_z-z,2))))
            msg.vz = v * ((t_z-z)/(math.sqrt(math.pow(t_x-x,2)+math.pow(t_y-y,2)+math.pow(t_z-z,2))))
    
        diff_x = t_x-x
        diff_y = t_y-y

        if(yaw!=0): ## heading 바꾸며 비행
            msg.yaw = math.atan2(diff_y,diff_x)
            self.previous_yaw = msg.yaw
        else: ## heading 고정하며 비행
            msg.yaw = float(self.previous_yaw)

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def compensation_path_with_odom(self): ##odom 으로 받은 xyz로 previous_waypoint 갱신. 경로 보정
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

    def goto_waypoint(self, to_x, to_y, to_z, velo, yaw): ## ? 원주비행 시에만 쓰는 걸까? ## 어디론가 갈 때 항상 씀
        if(self.is_new_go == 1): ## 로그 찍으려고 .. 
            self.get_logger().info(f"To {[to_x, to_y, to_z]}, at {velo}m/s")
            self.is_new_go = 0

        ## 목표점 설정
        x = to_x  
        y = to_y
        z = to_z
        #self.publish_offboard_control_heartbeat_signal(True) ## ? ture? position ## 삭제..
        self.distance_target = math.sqrt(math.pow(x-self.vehicle_odom.x,2) + math.pow(y-self.vehicle_odom.y,2) + math.pow((z-self.vehicle_odom.z),2)) ## target과의 떨어진 거리
        
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD: ## offboard 모드일 때 시그널을 보낸다..
            self.publish_offboard_control_heartbeat_signal(False) ## velocity 로 쭉 제어할 거야~ ##이게 여기 있으면 안  될 것 같은 .. .. .. ??


            # if ((self.distance_target < self.correction_range) | (self.setpoint_mode)):
            #     self.setpoint_mode = True
            #     self.publish_offboard_control_heartbeat_signal(False)
            #     self.stable_depart_publish(float(x), float(y), float(z), velo, yaw)
            # else:
            #     self.publish_offboard_control_heartbeat_signal(False)
            #     self.publish_velocity_setpoint(float(x), float(y), float(z), velo, yaw)

        if (self.distance_target < self.waypoint_range) & (self.waypoint_count != (self.waypoint_num-1)): ## waypoint_range 안에 들어오고 다음 wpt 가 존재한다면
            #self.setpoint_mode = True ## ? : 이건 true네.. 이제 안 씀
            self.wait_in_waypoint += 1
            if (self.wait_in_waypoint == 10): ## ? 10으로 줄인 이유가 뭘까 ##빠르게 하고 싶었어
                self.previous_waypoint[0] = self.vehicle_odom.x
                self.previous_waypoint[1] = self.vehicle_odom.y
                self.previous_waypoint[2] = self.vehicle_odom.z

                self.get_logger().info(f"{[to_x, to_y, to_z]}, departed !! ")

                self.is_new_go = 1
                self.is_departed = 1
                self.wait_in_waypoint = 0
                # self.setpoint_mode = False

        if (self.offboard_setpoint_counter % 45 == 0):  #timer_callback 주기 1/15라고 가정, 3초마다 보정 
            #self.get_logger().info(f"** path correction with odom **")
            self.compensation_path_with_odom()

    def circle_path_publish(self, t_x: float, t_y: float, t_z: float, w:float, radius):
        msg = TrajectorySetpoint() 
        pi = math.pi
        diff_x = self.vehicle_odom.x - t_x
        diff_y = self.vehicle_odom.y - t_y
        if(math.sqrt(pow(diff_x,2) + pow(diff_y,2)) > (radius + 0.5)): ## circle 경로로 도입 시 부드럽게 넘어가도록..
            if(self.is_go_to_center == 0): ## 원주 비행을 시작할 때, initial_theta2 를 설정하려고
                #self.get_logger().info(f" going to central point of circle ")
                self.initial_theta2 = math.atan2(diff_y, diff_x)
                self.is_go_to_center = 1 ## 다시 돌려놓음
            self.goto_waypoint(t_x + math.cos(self.initial_theta2)*radius, t_y + math.sin(self.initial_theta2)*radius, t_z, self.waypoint_velocity[self.waypoint_count], 1) ## circle 경로 도입점으로 가라
            return
        elif(math.sqrt(pow(diff_x,2) + pow(diff_y,2)) < (radius - 0.5)): 
            if(self.is_go_to_circle_point == 0):
                self.initial_theta2 = math.atan2(diff_y, diff_x)
            self.goto_waypoint(t_x+math.cos(self.initial_theta2) * radius, t_y+math.sin(self.initial_theta2)*radius, t_z, self.waypoint_velocity[self.waypoint_count], 1) ## 마찬가지..
            return
        self.publish_offboard_control_heartbeat_signal(True) ## position 제어
        if(self.initial_theta == -1): ## initial_theta 값에 저장된 것이 없을 때
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
        if (self.waypoint_count == 2 or self.waypoint_count == 3 or self.waypoint_count == 7 or self.waypoint_count == 8): 
        ##         wpt 2로 갈 때/          missionP 2로 갈 때 / comeback / wpt 2로 갈 때/         missionP 1로 갈 때
            self.is_mission_ladder += 1
        if (self.waypoint_count == 5 and self.is_mission_delivery == 0): ## wpt3을 지났을 때 & 배달을 아직 안 했을 때
            self.is_mission_delivery = 1 ## 배달 중, 배달 후

    def mission_ladder(self):

        if(self.is_mission_started == 1): ## (wpt 도착 후 다음 wpt로 갈 때 == is_mission_started)
            se1 = SE(self.real_obstacle_list) ## 장애물 정보를 SE 알고리즘 돌려서 list 추가
            self.get_logger().info(f" {[self.waypoint_list[self.waypoint_count][0], self.waypoint_list[self.waypoint_count][1], self.waypoint_list[self.waypoint_count][2]]}로 가기 위한 SE 알고리즘 경로를 생성합니다. ")
            self.waypoint_for_ladder = se1.make_final_path(self.waypoint_list[self.waypoint_count-1][0], self.waypoint_list[self.waypoint_count-1][1], self.waypoint_list[self.waypoint_count][0], self.waypoint_list[self.waypoint_count][1])
            ## ^ make_final_path(self, now_x, now_y, target_x, target_y)
            for i in self.waypoint_for_ladder: ## SE로 나온 via point들 저장, [xyz]로
                i.append(self.waypoint_list[self.waypoint_count][2]) ## SE 알고리즘을 통해 나온 점들은 z값이 없으므로 waypoint_for_ladder 에 waypoint_list[]의 z값 추가
            self.is_mission_started = 0 ## ???? /초기화 하는 거임.. 다음 미션 수행하려고...


        self.goto_waypoint(self.waypoint_for_ladder[self.ladder_mission_count][0], self.waypoint_for_ladder[self.ladder_mission_count][1], self.waypoint_for_ladder[self.ladder_mission_count][2], 
                           self.waypoint_velocity[self.waypoint_count], 1) ## 

        if(self.is_departed == 1):
            self.ladder_mission_count += 1 ## ladder 미션에서의 점들 count
            self.is_departed = 0 ## 도착 판단값 초기화

        if(self.ladder_mission_count == len(self.waypoint_for_ladder)): ## ladder_mission이 끝나면
            self.is_mission_started = 0 ## 시작값 0
            self.ladder_mission_count = 0 ## 초기화
            self.is_mission_ladder_finished = 1 ## SE point를 다 통과했을 때
    
    def make_points_for_contest(self, waypoint_1, waypoint_2, waypoint_3): 
        distance_w1_w2 = math.sqrt(pow(waypoint_1[0]-waypoint_2[0],2)+pow(waypoint_1[1]-waypoint_2[1],2)+pow(waypoint_1[2]-waypoint_2[2], 2))
        distance_w2_w3 = math.sqrt(pow(waypoint_2[0]-waypoint_3[0],2)+pow(waypoint_2[1]-waypoint_3[1],2)+pow(waypoint_2[2]-waypoint_3[2], 2))
        mission_point_1 = [waypoint_2[0]-(8.5*((waypoint_2[0]-waypoint_1[0])/distance_w1_w2)), waypoint_2[1]-(8.5*((waypoint_2[1]-waypoint_1[1])/distance_w1_w2)), waypoint_1[2]] ## mission point 
        mission_point_2 = [waypoint_2[0]+(8.5*((waypoint_3[0]-waypoint_2[0])/distance_w2_w3)), waypoint_2[1]+(8.5*((waypoint_3[1]-waypoint_2[1])/distance_w2_w3)), waypoint_2[2]]
        self.waypoint_list = [waypoint_1, mission_point_1, waypoint_2, mission_point_2, waypoint_3, waypoint_3, mission_point_2, waypoint_2, mission_point_1, waypoint_1,  [waypoint_1[0], waypoint_1[1], 0]]
                            #   0                1            2             3               4            5               6              7             8                    9
        
    def mission_delivery(self):
        delivery_velocity = 0.8 ## crossbow_started > crossbow
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
        elif(self.is_delivery_going == 2):
            # to crossbow
            if(self.pizza_closed_point_distance==0):
                self.pizza_closed_point_distance = math.sqrt(pow(self.crossbow_location_confirmed[0]-self.crossbow_start_point[0],2)+pow(self.crossbow_location_confirmed[1]-self.crossbow_start_point[1],2))
            d = self.pizza_closed_point_distance
            self.goto_waypoint(self.crossbow_location_confirmed[0]+(3/d)*(self.crossbow_start_point[0]-self.crossbow_location_confirmed[0]), self.crossbow_location_confirmed[1]+(3/d)*(self.crossbow_start_point[1]-self.crossbow_location_confirmed[1]), self.crossbow_location_confirmed[2], delivery_velocity, 1)
            if(self.is_departed == 1):
                self.is_delivery_going = 3
                self.is_departed = 0
        elif(self.is_delivery_going == 3):
            self.get_logger().info(f" delivery finished ")
            self.is_mission_delivery = -1
    
    def cross_bow_detect(self):
        w = 0.01 ## radian
        if (self.is_delivery_started == 0): ## 아직 배달 미션 시작 전
            #self.goto_waypoint(self.waypoint_contest[2][0], self.waypoint_contest[2][1], -7, 1, 0)
            self.goto_waypoint(self.waypoint_contest[2][0],self.waypoint_contest[2][1],-7,1,0) ## (wpt3 이후) 고도 7m로 하강 
            if(self.is_departed == 1): ## wpt3의 고도 7m 지점에 도달하면
                self.is_departed = 0
                self.is_delivery_started = -1
        elif (self.is_delivery_started < 0):
            if(self.is_delivery_started == -1): ## wpt3의 고도 7m 지점에 도달했을 때
                self.theta_yaw = euler_from_quaternion(self.vehicle_odom.q[0], self.vehicle_odom.q[1], self.vehicle_odom.q[2], self.vehicle_odom.q[3]) ## 현 위치에 대한 theta값 설정
            self.theta_yaw += w ## 제자리 회전을 위한 w 합
  
            self.publish_yaw_with_hovering(self.waypoint_contest[2][0], self.waypoint_contest[2][1], -7, 1, self.theta_yaw) ##고도 7m로 제자리 회전
            self.is_delivery_started -= 1
            if(self.is_delivery_started < -(2*math.pi/w)): ##2pi/w : w만큼 회전하면 2pi가 되는 거임 // 한 바퀴 돌았는지 확인하는 것
                self.get_logger().info(f" balcony detected ") ## detected? 
                self.is_delivery_started = 1
            #(balcony 가장 가까운 위치 정보 subsribe)
            # self.balcony_location에 balcony 위치 저장
        else :
            #print(1)
            self.circle_path_publish(self.balcony_location[0], self.balcony_location[1], -7, 0.1, 6) ## 발코니 중심으로 반지름 6m circle 회전
            '''
            십자가 detected 정보 subscribe,
            if(십자가 detected):
                self.crossbow_showed_list.append([self.vehicle_odom.x, self.vehicle_odom.y, self.vehicle_odom.z]) 
             
            ''' 
            if (self.theta-self.initial_theta > math.pi *2 ): # 한 바퀴 돌기 ## 한 바퀴 돌고 이것저것 바꾸기
                self.circle_path = 0
                self.is_go_to_center = 0
                self.is_go_to_circle_point = 0
                self.is_crossbow_detected = 1        
                # for(i in self.crossbow_showed_list)
                    #self.crossbow_start_point[0] += i[0]
                    #self.crossbow_start_point[1] += i[1]
                # self.crossbow_start_point[0] = self.crossbow_start_point[0]/len(self.crossbow_showed_list)
                # self.crossbow_start_point[1] = self.crossbow_start_ponit[1]/len(self.crossbow_showed_list)

    def ladder_detect_flight(self):
        self.circle_path_publish(self.waypoint_contest[1][0], self.waypoint_contest[1][1], self.waypoint_contest[1][2], 0.07, 8.5) ## 8.5m 반지름으로 원주비행, 0.07 rad/s 천천히 회전 (but 기준은 없다..)
        if(self.theta-self.initial_theta > math.pi * 2): ## 회전 이후
            self.circle_path = 0
            self.is_go_to_center = 0
            self.is_ladder_detected = 1
            self.initial_theta = -1
        

    def timer_callback(self) -> None:
        
        #self.get_logger().info(f"local position {[self.vehicle_odom.x, self.vehicle_odom.y, self.vehicle_odom.z]}")
        """Callback function for the timer."""
        x = float(self.waypoint_list[self.waypoint_count][0])
        y = float(self.waypoint_list[self.waypoint_count][1])
        z = float(self.waypoint_list[self.waypoint_count][2])
        v = float(self.waypoint_velocity[self.waypoint_count])
        yaw = float(self.waypoint_yaw[self.waypoint_count])
        
        if self.offboard_setpoint_counter == 10: ## 처음, 토픽 10까지 발행 후 이륙
            self.engage_offboard_mode()
            self.arm()
            self.make_points_for_contest(self.waypoint_contest[0], self.waypoint_contest[1], self.waypoint_contest[2]) ## 시작 시 받은 wpt 3점을 코드에서 쓸 wpt 리스트에 넣음
        
        if(self.is_mission_ladder == 1 and self.waypoint_count == 2 and self.is_mission_ladder_finished == 0): ## ?? mw1에 도착해서 wpt 2로 가야 할 때 & wpt_count가 2일 때 (wpt 2로 갈 때) & ladder mission이 안 끝났을 때
            if(self.wait_time == 0): ## 처음에는 wait time이 0임
                self.wait_time = self.offboard_setpoint_counter  ## wait_time에 그때의 setpoint_count 저장. 이후 setpoint_count의 차이로 시간 경과 확인하기 위해

            if(self.offboard_setpoint_counter - self.wait_time < 10 and self.is_ladder_detected == 0): ## 토픽이 10번 발행되기 전 & 8.5m 원주비행 전
                self.publish_offboard_control_heartbeat_signal(False) ## velocity 제어
                ## 왜 wpt-1에서의 xyz를 target으로 갈까...? 정지비행 하는 거임. 기다리는 거임
                self.stable_depart_publish(self.waypoint_list[self.waypoint_count-1][0], self.waypoint_list[self.waypoint_count-1][1], self.waypoint_list[self.waypoint_count-1][2], self.waypoint_velocity[self.waypoint_count-1], 0)
                
                if (self.offboard_setpoint_counter - self.wait_time == 9): ## 차이가 9가 되도록 수정
                    self.wait_time = -1 
                
                # if (self.offboard_setpoint_counter == 9): ## 차가 9인 게 아니고 offboard_setpoint_counter 가 9???
                #     self.wait_time = -1 

            elif(self.is_ladder_detected == 0): ##사다리 탐지가 안 되면
                self.ladder_detect_flight() ## 사다리 탐지 비행 (r8.5 원주비행)

                if(self.is_ladder_detected == 1): ## 사다리 탐지했으면 (r8.5 원주비행 완료 후 mw1에 다시 왔을 때)
                    self.wait_time = self.offboard_setpoint_counter ## wait_time에 setpoint_count 저장
            elif(self.is_ladder_detected == 1 and self.wait_time != -2): ## 사다리 탐지 완료 & wait_time 이 -2가 아닐 때 (mw1에서 토픽 10번 발행 전)
                self.publish_offboard_control_heartbeat_signal(False) ## velocity 제어
                ## 정지, 기다림 
                self.stable_depart_publish(self.waypoint_list[self.waypoint_count-1][0], self.waypoint_list[self.waypoint_count-1][1], self.waypoint_list[self.waypoint_count-1][2], self.waypoint_velocity[self.waypoint_count-1], 0)
                if(self.offboard_setpoint_counter - self.wait_time == 10): ## 토픽 10번 발행되었다면 (토픽 10번 발행동안 정지 후)
                    self.wait_time = -2 ## SE 알고리즘으로 넘어감 (else 부분)
            else:
                self.mission_ladder() ## SE 알고리즘 짜고 wpt 2로 가자~
        
        elif(self.is_mission_ladder == 2 and self.waypoint_count == 3 and self.is_mission_ladder_finished == 0): ## (go) MPoint2로 갈 때, se 점들을 다 통과하지 못했을 때, wpt2 도달 후 이제 mp2로 간다~
            self.mission_ladder()

        elif(self.is_mission_ladder == 3 and self.waypoint_count == 7 and self.is_mission_ladder_finished == 0): ## (back) wpt2로 갈 때, 사다리 미션 못 했을 때 : 사다리 미션 수행 ; mp2에서 wpt2로 간다~
            self.mission_ladder()
        
        elif(self.is_mission_ladder == 4 and self.waypoint_count == 8 and self.is_mission_ladder_finished == 0): ## (back) mp1로 갈 때, 사다리 미션 못 했을 때? : wpt2에서 mp1 로 간다~ 
            self.mission_ladder()

        elif(self.is_mission_delivery == 1): ## wpt3을 지남 & 배달 아직 안 함
            self.mission_delivery()
            self.is_departed = 0
        
        else:
            self.goto_waypoint(x, y, z, v, yaw) ## 미션 상황이 아닐 때; (x, y, z)로 감
       
        if(self.is_departed == 1 and (self.waypoint_count < self.waypoint_num)): # next waypoint update ## wpt에 도달했고 다음 wpt가 존재
            self.waypoint_count += 1
            self.is_mission_ladder_finished = 0 
            self.is_mission_started = 1 ## 미션 초기값 설정, 끝났을 때 0으로 설정, wpt 지났을 때 미션 종료?
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
