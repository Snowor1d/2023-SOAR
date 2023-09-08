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
from re import T
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

#PX4' quaternion yaw 
def plus_radian(w, plus_w):
    w += plus_w
    if(w>2*math.pi):
        w-=2*math.pi
    if(w<0):
        w+=2*math.pi
    return w 


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
        self.end_p = 0
        self.w_count = 0 

        self.now_yaw = 0


        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.vehicle_odom = VehicleOdometry()

        self.waypoint_contest = [[-1,-1,-15], [1.65, 50, -15], [16, 154, -15]] ##대회에서 주는 wpt 3개
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
        self.previous_yaw = 0
        self.distance_target = 0
        self.is_go_to_center = 0 ## ?
        self.stable_counter = 0 ## ?
        self.stable_odom = [0, 0, 0]
        self.is_go_to_circle_point = 0
        self.stayed_finished = 0
        self.change_yaw = -1
        self.one_direction_time = 0
        self.deacc_num = 0
        
        self.real_ground_z = 0
        self.real_ground_x = 0
        self.real_ground_y = 0
        self.real_ground_yaw = 0
        self.ground_z = 0
        self.ground_x = 0
        self.ground_y = 0
        self.ground_yaw = 0
        self.ground_initial_num = 0
        self.initial_theta3 = -10
        self.wait_time = 0
        self.any_direction_time = 0
    
        self.is_new_go = 0 ## 다음 wpt가 존재하는지
        self.is_departed = 0 ## wpt에 도달했는지 
        
        
        #variables for ladder
        self.is_mission_ladder = 0
        self.is_mission_delivery = 0
        self.is_mission_started = 0
        self.ladder_mission_count = 0
        self.is_ladder_mission_finished = 0
        self.is_ladder_detected = 0

        self.real_ladder_list = [[0, 0, 1.5], [0, 0, 1.5]]
        self.waypoint_for_ladder = [] ## ladder 미션 시/ SE Algorithm을 통해 생성되는 point 저장 list 
    
        self.theta = 0
        self.initial_theta = -1 ## theta 초기 설정값.. 실제 존재하지 않는 값으로 설정
        self.initial_theta2 = -1 
        self.circle_path = 1
        self.sub_positions = []
    

        self.confirmed_ladder_num = 0

        self.ladder_list_guess = [[0, 0 ,1.5], [0,0,1.5]]
        self.point_list = []
        self.callback_flag = 0 


        #variables for delivery
        self.is_mission_delivery = 0
        self.is_crossbow_detected = 0
        self.crossbow_location = [0, 0, -8] ## 드론의 배란다 원주 비행 시 처음 들어오는 + 위치 저장 ; NED 좌표계 상의 값이므로 변화 x ## 이게 필요하지는 않은 듯? junalee의 요청..
        self.crossbow_location_confirmed = [0, 0, -8] ## crossbow_start_position에서 3m 간격 위치까지 이동하며 crossbow_location을 3개의 평균값으로 보정?? ## topic 세 번 발행 때마다 들어온 위치 값의 평균

        self.is_delivery_started = 0
        self.theta_yaw = 0
        self.crossbow_showed_list = [] #이중 리스트
        self.crossbow_start_point = [0, 0, -7] ## ? ; 계산으로 얻는 것 아니었나? 어떻게 처음부터 알고 있을까 ##임의로 설정
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
            #print("추측한 사다리 위치 : ")
            #print(self.ladder_list_guess)

    def detect_balcony(self):
        qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
        )
        grid_cells_subscriber2 = self.create_subscription(
            GridCells, '/map_talker/gridcells', self.PCL_callback2, 10)
        #print(3)
        #print(self.balcony_confirmed_x)
        #print(self.balcony_confirmed_y)
    
        return [self.balcony_confirmed_x, self.balcony_confirmed_y]
    
    def PCL_callback2(self, msg):
        if(self.offboard_setpoint_counter % 4 !=0):
            return
        if(len(msg.cells)!=1):
            print("엥???")
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
        cross_location_subscriber = self.create_subscription(
            Point, 's2s_result', self.cross_callback, 10)
        return self.cross_location

    def cross_callback(self, msg):
        self.cross_location = [msg.x, msg.y, msg.z]

    def vehicle_odom_callback(self, vehicle_odom):
        self.vehicle_odom = vehicle_odom
        self.now_yaw = euler_from_quaternion(self.vehicle_odom.q[0], self.vehicle_odom.q[1], self.vehicle_odom.q[2], self.vehicle_odom.q[3]) ## 현 위치에 대한 theta값 설정
        

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

    def ladder_detect_on(self): # detect_object 임포트해서 clustering 평균 낸 사다리 두개 찾는 함수 
        guess_ladder = self.detect_ladder()
        if(len(guess_ladder) != 2): #ladder 정보가 2개가 아니다? 뭔가 이상한 상황. 서둘러 함수 빠져나와야 함 
            return
        ladder1_x = guess_ladder[0][0]
        ladder1_y = guess_ladder[0][1]
        ladder2_x = guess_ladder[1][0]
        ladder2_y = guess_ladder[1][1] # clustering 한 결과 ladder1, ladder2에 저장
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
            ladder2_y = temp_y             # ladder1, ladder2가 섞이지 않도록 정리
                                           # 부연설명 : 이거 갑자기 왜하냐??면 드론이 사다리를 돌면서 ladder1 정보는 ladder1끼리, ladder2 정보는 ladder2끼리 합쳐지고 평균이 내져야함. 근데 무엇이 ladder1이고 무엇이 ladder2 인지
                                           # 확신할 수 없잖아? 그래서 이미 추정중인 ladder1, ladder2의 위치를 보고 새로 들어온 ladder의 정보를 가까운데로 끼어 맞추는 거임 

        if(distance<9 and distance>5 and distance_waypoint<2):
            if(self.confirmed_ladder_num==0):
                self.real_ladder_list[0][0] = ladder1_x
                self.real_ladder_list[0][1] = ladder1_y
                self.real_ladder_list[1][0] = ladder2_x
                self.real_ladder_list[1][1] = ladder2_y # ladder 정보가 처음 들어왔으면, 일단 저장
                self.confirmed_ladder_num+=1
            else:
                self.real_ladder_list[0][0] = (self.real_ladder_list[0][0]) * (self.confirmed_ladder_num/(self.confirmed_ladder_num+1)) + ladder1_x/(self.confirmed_ladder_num+1)
                self.real_ladder_list[0][1] = (self.real_ladder_list[0][1]) * (self.confirmed_ladder_num/(self.confirmed_ladder_num+1)) + ladder1_y/(self.confirmed_ladder_num+1)
                self.real_ladder_list[1][0] = (self.real_ladder_list[1][0]) * (self.confirmed_ladder_num/(self.confirmed_ladder_num+1)) + ladder2_x/(self.confirmed_ladder_num+1)
                self.real_ladder_list[1][1] = (self.real_ladder_list[1][1]) * (self.confirmed_ladder_num/(self.confirmed_ladder_num+1)) + ladder2_y/(self.confirmed_ladder_num+1)
                self.confirmed_ladder_num += 1
                print("ladder 위치 보정됨 : " + str(self.real_ladder_list))
                # ladder 좌표의 평균 내기. 새로 들어온 ladder의 정보, 원래 평균냈던 ladder의 정보, 이제 까지 평균내는데 사용했던 ladder 정보의 개수를 이용하면 새롭게 평균을 정의할 수 있겠지? 
    

    def balcony_detect_on(self):
        guess_balcony = self.detect_balcony()
        if(len(guess_balcony)==0): # balcony 정보가 안들어왔었다면 서둘러 도망치기 
            return 
        if(guess_balcony[0]==0 and guess_balcony[1]==0):
            return
        #print(1)
        #print(guess_balcony)
        if(self.confirmed_balcony_num ==0): #만약 balcony 정보가 들어온게 없다면, 새롭게 정의
            self.confirmed_balcony_location[0] = guess_balcony[0]
            self.confirmed_balcony_location[1] = guess_balcony[1]
            self.confirmed_balcony_num += 1 
        else:
            self.confirmed_balcony_location[0] = (self.confirmed_balcony_location[0]) * (self.confirmed_balcony_num/(self.confirmed_balcony_num+1)) + guess_balcony[0]/(self.confirmed_balcony_num+1)
            self.confirmed_balcony_location[1] = (self.confirmed_balcony_location[1]) * (self.confirmed_balcony_num/(self.confirmed_balcony_num+1)) + guess_balcony[1]/(self.confirmed_balcony_num+1)
            self.confirmed_balcony_num += 1 #마찬가지로 balcony 좌표 평균 내기
            print("balcony 위치 보정됨 : " + str(self.confirmed_balcony_location))
        #print(2)
        #print(self.confirmed_balcony_location[0])
        #print(self.confirmed_balcony_location[1])
    def cross_detect_on(self):
        if(self.is_crossbow_detected == 0): #왜 self.is_crossbow_detected가 0일떄와 1일때를 구분했냐?? self.is_crossbow_detected가 0일 때 -> 드론이 십자가와 수직으로 정렬 위치를 아직 못찾은 상황, 1일 때는 정렬 위치를 찾은 상황임
                                            #내가 저번에 정렬 위치를 찾은 상황과 못찾은 상황에서 cross 정보를 다르게 저장할거라 했잖아? 정확도 이슈 때문에.. 그거 ㅇㅇ
            guess_crossbow = self.detect_crossbow()
            if (len(guess_crossbow) == 0 or (guess_crossbow[0]==0 and guess_crossbow[1]==0)):
                return 0 #근데 갑자기 cross 정보가 사실 안들어왔었다면 서둘러 도망쳐야겠지?
            self.crossbow_location[0] = guess_crossbow[0]
            self.crossbow_location[1] = guess_crossbow[1]
            self.crossbow_location[2] = guess_crossbow[2] # 정렬 위치를 못찾은 상황에서는 (아직 발코니를 돌고 있을 때) 십자가 위치 대충 저장~ 왜냐하면 이때 중요한 것은 '십자가가 발견되었을 때의 나의 위치' 거든. 왜냐? 정렬 위치 먼저 찾아야하기 때문

        if(self.is_crossbow_detected == 1): # 정렬 위치 발견 -> 본격적으로 십자가 디텍팅 
            guess_crossbow = self.detect_crossbow()
            if(len(guess_crossbow)==0 or (guess_crossbow[0]==0 and guess_crossbow[1]==0)): 
                return 0
            if(self.confirmed_crossbow_num == 0):
                self.crossbow_location_confirmed[0] = guess_crossbow[0]
                self.crossbow_location_confirmed[1] = guess_crossbow[1]
                self.crossbow_location_confirmed[2] = guess_crossbow[2]
                self.confirmed_crossbow_num +=1
            else:
                self.crossbow_location_confirmed[0] = (self.crossbow_location_confirmed[0]) * (self.confirmed_crossbow_num/(self.confirmed_crossbow_num+1)) + guess_crossbow[0]/(self.confirmed_crossbow_num+1)
                self.crossbow_location_confirmed[1] = (self.crossbow_location_confirmed[1]) * (self.confirmed_crossbow_num/(self.confirmed_crossbow_num+1)) + guess_crossbow[1]/(self.confirmed_crossbow_num+1)
                self.crossbow_location_confirmed[2] = (self.crossbow_location_confirmed[2]) * (self.confirmed_crossbow_num/(self.confirmed_crossbow_num+1)) + guess_crossbow[2]/(self.confirmed_crossbow_num+1)
                self.confirmed_crossbow_num += 1 # 계속 십자가 위치 정보 평균 내기
                print("cross 좌표 보정됨 : " + str(self.crossbow_location_confirmed))
            return 1

                


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
        xy_distance = math.sqrt(math.pow(t_x-self.previous_waypoint[0],2)+math.pow(t_y-self.previous_waypoint[1], 2))
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
            if(v < 0.65): ## range 안에서 v < 0.65 일 때: ##0.65는 실험적으로 설정한 값
                self.publish_velocity_setpoint(t_x, t_y, t_z, v*(self.distance_target/self.correction_range), 0) ## 선형적으로 속도 감소
                return
            else:
                self.publish_velocity_setpoint(t_x, t_y, t_z, v*(pow(self.distance_target,2)/pow(self.correction_range,2)),0) ## 이후 
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
                ##? 얘는 return을 안 하네
        

    def publish_velocity_setpoint(self, t_x: float, t_y: float, t_z:float, v:float, yaw:float): ## (t_x, t_y, t_z)에 v의 속도로 가도록
        plus_yaw = 0.07

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
        #print(self.change_yaw)    

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
                                                               
        self.publish_offboard_control_heartbeat_signal(False) ## ? true? position ## 삭제..
        self.distance_target = math.sqrt(math.pow(x-self.vehicle_odom.x,2) + math.pow(y-self.vehicle_odom.y,2) + math.pow((z-self.vehicle_odom.z),2)) ## target과의 떨어진 거리
        
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if (self.distance_target < self.correction_range):
                self.stable_depart_publish(float(x), float(y), float(z), velo, yaw)
            else:
                self.publish_velocity_setpoint(float(x), float(y), float(z), velo, yaw)
            
        if (self.distance_target < self.waypoint_range) & (self.waypoint_count != (self.waypoint_num-1)): ## waypoint_range 안에 들어오고 다음 wpt 가 존재한다면

            self.wait_in_waypoint += 1
            if (self.wait_in_waypoint == 10): ## ? 10으로 줄인 이유가 뭘까 ##빠르게 하고 싶었어
                self.previous_waypoint[0] = self.vehicle_odom.x
                self.previous_waypoint[1] = self.vehicle_odom.y
                self.previous_waypoint[2] = self.vehicle_odom.z

                self.get_logger().info(f"{[to_x, to_y, to_z]}, departed !! ")

                self.is_new_go = 1
                self.is_departed = 1
                self.wait_in_waypoint = 0

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
    
    def mission_check(self):
        if (self.waypoint_count == 2 or self.waypoint_count == 3 or self.waypoint_count == 7 or self.waypoint_count == 8): 
        ##         wpt 2로 갈 때/          missionP 2로 갈 때 / comeback / wpt 2로 갈 때/         missionP 1로 갈 때
            self.is_mission_ladder += 1
        if (self.waypoint_count == 5 and self.is_mission_delivery == 0): ## wpt3을 지났을 때 & 배달을 아직 안 했을 때
            self.get_logger().info(" departed at wpt 3 and delivering ")
            self.is_mission_delivery = 1 ## 배달 중, 배달 후


    def mission_ladder(self):

        if(self.is_mission_started == 1): ## (wpt 도착 후 다음 wpt로 갈 때 == is_mission_started)
            se1 = SE(self.real_ladder_list) ## 장애물 정보를 SE 알고리즘 돌려서 list 추가
            self.sub_positions = make_orth_points(self.real_ladder_list[0][0], self.real_ladder_list[0][1], self.real_ladder_list[1][0], self.real_ladder_list[1][1], 2)
            arrange_shortest_point(self.waypoint_list[self.waypoint_count-1][0], self.waypoint_list[self.waypoint_count-1][1], self.sub_positions)
            self.get_logger().info(f" {[self.waypoint_list[self.waypoint_count][0], self.waypoint_list[self.waypoint_count][1], self.waypoint_list[self.waypoint_count][2]]}로 가기 위한 SE 알고리즘 경로를 생성합니다. ")
            
            if(self.waypoint_count == 2 or self.waypoint_count==7):
                self.waypoint_for_ladder = se1.make_final_path(self.waypoint_list[self.waypoint_count-1][0], self.waypoint_list[self.waypoint_count-1][1], self.sub_positions[0], self.sub_positions[1])
                self.waypoint_for_ladder.append([self.sub_positions[0], self.sub_positions[1]])
            elif(self.waypoint_count == 3 or self.waypoint_count == 8):
                self.waypoint_for_ladder = se1.make_final_path(self.sub_positions[2], self.sub_positions[3], self.waypoint_list[self.waypoint_count][0], self.waypoint_list[self.waypoint_count][1])
                self.waypoint_for_ladder.insert(0, [self.sub_positions[2], self.sub_positions[3]])
            
            ## ^ make_final_path(self, now_x, now_y, target_x, target_y)
            for i in self.waypoint_for_ladder: ## SE로 나온 via point들 저장, [xyz]로
                i.append(self.waypoint_list[self.waypoint_count][2]) ## SE 알고리즘을 통해 나온 점들은 z값이 없으므로 waypoint_for_ladder 에 wpublish_velocityaypoint_list[]의 z값 추가
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
            self.cross_detect_on() # 본격적으로 십자가 위치 정보 저장

            if(self.crossbow_location_confirmed[0]==0 and self.crossbow_location_confirmed[1]==0):
                self.crossbow_location_confirmed = self.emergency_crossbow_location
                self.get_logger().info(f" fail to detect cross ")

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
            self.balcony_detect_on() # 들어오는 포인트 클라우드 정보 subscribe 해서 발코니 정보 저장
  
            self.publish_yaw_with_hovering(self.waypoint_contest[2][0], self.waypoint_contest[2][1], -7, 1, self.theta_yaw) ##고도 7m로 제자리 회전
            self.is_delivery_started -= 1
            if(self.is_delivery_started < -(2*math.pi/w)): ##2pi/w : w만큼 회전하면 2pi가 되는 거임 // 한 바퀴 돌았는지 확인하는 것
                if(self.confirmed_balcony_location[0]==0 and self.confirmed_balcony_location[1]==0):
                    self.get_logger().info(f" balcony detect failed. use emergency balcony location ")
                    self.confirmed_balcony_location = self.emergency_balcony_location[:]
                    self.is_delivery_started = 1
                else:
                    self.get_logger().info(f" balcony detected ") ## detected? 
                    self.is_delivery_started = 1
            #(balcony 가장 가까운 위치 정보 subsribe)
            # self.balcony_location에 balcony 위치 저장
        else :
            #print(1)
            self.circle_path_publish(self.confirmed_balcony_location[0], self.confirmed_balcony_location[1], -7, 0.1, 8.5) ## 발코니 중심으로 반지름 6m circle 회전
          
            if(self.cross_detect_on()): # 왜 갑자기 if문에 넣냐면.. 이 함수가 십자가가 보이지 않는다면 return 0을 하기 때문. 만약 십자가가 보인다면 함수 돌아가면서 십자가 정보 저장
                self.crossbow_showed_list.append([self.vehicle_odom.x, self.vehicle_odom.y, self.vehicle_odom.z]) # 십자가를 돌며 십자가가 보일 때의 드론의 위치 정보 저장 
             

            if (self.theta-self.initial_theta > math.pi *2 ): # 한 바퀴 돌기 ## 한 바퀴 돌고 이것저것 바꾸기
                self.circle_path = 0
                self.is_go_to_center = 0
                self.is_go_to_circle_point = 0
                self.is_crossbow_detected = 1

                crossbow_showed_list_num = len(self.crossbow_showed_list)
                self.crossbow_start_point = [0, 0, 0]
                for i in self.crossbow_showed_list:
                    self.crossbow_start_point[0] += i[0]
                    self.crossbow_start_point[1] += i[1]
                    self.crossbow_start_point[2] += i[2]
                if(crossbow_showed_list_num != 0):
                    self.crossbow_start_point[0] = self.crossbow_start_point[0]/crossbow_showed_list_num
                    self.crossbow_start_point[1] = self.crossbow_start_point[1]/crossbow_showed_list_num
                    self.crossbow_start_point[2] = self.crossbow_start_point[2]/crossbow_showed_list_num # 평균내서 정렬 위치 찾기 !!!
                if(self.crossbow_start_point[0]==0 and self.crossbow_start_point[1]==0):
                    self.get_logger().info(f" fail to find proper location to align drone with crossbow. use emergency location ")
                    self.crossbow_start_point = self.emergency_crossbow_started_location 

    def ladder_detect_flight(self):
        self.circle_path_publish(self.waypoint_contest[1][0], self.waypoint_contest[1][1], self.waypoint_contest[1][2], 0.07, 8.5) ## 8.5m 반지름으로 원주비행, 0.07 rad/s 천천히 회전 (but 기준은 없다..)
        #if( (self.offboard_setpoint_counter % 3 ==0) and (self.offboard_setpoint_counter % 5 == 0)):
        self.ladder_detect_on()
        if(self.theta-self.initial_theta > math.pi * 2): ## 회전 이후
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
        self.is_mission_started = 1 ## 미션 초기값 설정, 끝났을 때 0으로 설정, wpt 지났을 때 미션 종료?
        self.is_departed = 0
        self.mission_check()
    def print_odom(self):
        self.get_logger().info(f" odom x y z yaw : {self.vehicle_odom.x, self.vehicle_odom.y, self.vehicle_odom.z, euler_from_quaternion(self.vehicle_odom.q[0], self.vehicle_odom.q[1], self.vehicle_odom.q[2], self.vehicle_odom.q[3])} ")

    def move_only_one_direction(self, direction, v, publish_num):
        self.publish_offboard_control_heartbeat_signal(False)
        msg = TrajectorySetpoint()
        self.deacc_num = 10
        self.one_direction_time +=1
        if(self.one_direction_time == publish_num):
            self.is_departed = 1
            return
        if(publish_num - self.one_direction_time < self.deacc_num):
            v = v * (publish_num-self.one_direction_time)/self.deacc_num
        if (direction == 'x'):
            msg.vx = float(v)
            msg.vy = float(0)
            msg.vz = float(0)
        elif(direction == '-x'):
            msg.vx = float(-v)
            msg.vy = float(0)
            msg.vz = float(0)

        elif (direction == 'y'):
            msg.vx = float(0)
            msg.vy = float(v)
            msg.vz = float(0)

        elif (direction == '-y'):
            msg.vx = float(0)
            msg.vy = float(-v)
            msg.vz = float(0)

        elif (direction == 'z'):
            msg.vx = float(0)
            msg.vy = float(0)
            msg.vz = float(-v)

        elif (direction == '-z'):
            msg.vx = float(0)
            msg.vy = float(0)
            msg.vz = float(v)

        elif (direction == 'n'):
            msg.vx = float(0)
            msg.vy = float(0)
            msg.vz = float(0)
        if(self.vehicle_odom.z < self.real_ground_z-3 and direction != '-z'):
            if(self.vehicle_odom.z <= self.real_ground_z-4):
                msg.vz = 0.5*abs(self.real_ground_z-4-self.vehicle_odom.z)
            else :
                msg.vz = -0.5*abs(self.real_ground_z-4-self.vehicle_odom.z)
        
        elif(self.vehicle_odom.x > self.real_ground_x+10 or self.vehicle_odom.x < self.real_ground_x-10):
            msg.vz = float(0)
            msg.vx = float(0)
            msg.vy = float(0)
        elif(self.vehicle_odom.y > self.real_ground_y+10 or self.vehicle_odom.y < self.real_ground_y-10):
            msg.vz = float(0)
            msg.vx = float(0)
            msg.vy = float(0)
    
        
        msg.x = math.nan
        msg.y = math.nan
        msg.z = math.nan


        yaw_flag = 0
        if(msg.vx == 0 and msg.vy == 0):
            msg.yaw = math.nan
            yaw_flag=1
        else :
            msg.yaw = float(math.atan2(msg.vy, msg.vx))
        plus_yaw = 0.07
        pi = math.pi
        if(yaw_flag == 0):
            t_yaw = math.atan2(msg.vy, msg.vx)
            if(t_yaw-self.now_yaw>=4*plus_yaw and t_yaw-self.now_yaw<pi):
                self.one_direction_time -= 1
                if(self.change_yaw==-1):
                    self.change_yaw = self.now_yaw
                msg.vx = float(0)
                msg.vy = float(0)
                msg.vz = float(0)
                self.change_yaw = plus_radian(self.change_yaw, plus_yaw) 
            elif(t_yaw-self.now_yaw>=pi and t_yaw-self.now_yaw<=2*pi-4*plus_yaw):
                if(self.change_yaw == -1):
                    self.change_yaw = self.now_yaw
                self.one_direction_time -= 1
                msg.vx = float(0)
                msg.vy = float(0)
                msg.vz = float(0)
                self.change_yaw = plus_radian(self.change_yaw, -plus_yaw)
            elif(self.now_yaw-t_yaw>4*plus_yaw and self.now_yaw-t_yaw<=pi):
                self.one_direction_time -= 1
                if(self.change_yaw == -1):
                    self.change_yaw = self.now_yaw
                msg.vx = float(0)
                msg.vy = float(0)
                msg.vz = float(0)
                self.change_yaw = plus_radian(self.change_yaw, -plus_yaw)
            elif(self.now_yaw-t_yaw>pi and self.now_yaw-t_yaw<=2*pi-4*plus_yaw):
                self.one_direction_time -= 1
                if(self.change_yaw == -1):
                    self.change_yaw = self.now_yaw
                msg.vx = float(0)
                msg.vy = float(0)
                msg.vz = float(0)
                self.change_yaw = plus_radian(self.change_yaw, plus_yaw)
            else:
                self.change_yaw = t_yaw
                
            msg.yaw = float(self.change_yaw) 
        
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.trajectory_setpoint_publisher.publish(msg)
        if(self.offboard_setpoint_counter %5 ==0):
            self.get_logger().info(f" vx, vy, vz, yaw oper : {msg.vx, msg.vy, msg.vz, msg.yaw} \n")

    def move_any_direction(self, direction, v, publish_num):
        msg = TrajectorySetpoint()
        self.deacc_num = 10
        self.any_direction_time +=1
        if(self.any_direction_time == publish_num):
            self.is_departed = 1
            return
        if(publish_num - self.one_direction_time < self.deacc_num):
            v = v * (publish_num-self.one_direction_time)/self.deacc_num
        direction = direction*math.pi/180
        msg.vx = v * math.cos(direction)
        msg.vy = v * math.sin(direction)

        if(self.vehicle_odom.z < self.real_ground_z-3 and direction != '-z'):
            if(self.vehicle_odom.z <= self.real_ground_z-4):
                msg.vz = 0.5*abs(self.real_ground_z-4-self.vehicle_odom.z)
            else :
                msg.vz = -0.5*abs(self.real_ground_z-4-self.vehicle_odom.z)
        
        elif(self.vehicle_odom.x > self.real_ground_x+10 or self.vehicle_odom.x < self.real_ground_x-10):
            msg.vz = float(0)
            msg.vx = float(0)
            msg.vy = float(0)
        elif(self.vehicle_odom.y > self.real_ground_y+10 or self.vehicle_odom.y < self.real_ground_y-10):
            msg.vz = float(0)
            msg.vx = float(0)
            msg.vy = float(0)    
        msg.x = self.vehicle_odom.x
        msg.y = self.vehicle_odom.y
        msg.z = math.nan

        yaw_flag = 0
        if(msg.vx == 0 and msg.vy == 0):
            msg.yaw = math.nan
            yaw_flag=1
        else :
            msg.yaw = float(math.atan2(msg.vy, msg.vx))
        plus_yaw = 0.07
        pi = math.pi
        if(yaw_flag == 0):
            t_yaw = math.atan2(msg.vy, msg.vx)
            if(t_yaw-self.now_yaw>=4*plus_yaw and t_yaw-self.now_yaw<pi):
                self.publish_offboard_control_heartbeat_signal(True)
                self.any_direction_time -= 1
                if(self.change_yaw==-1):
                    self.change_yaw = self.now_yaw
                msg.vx = float(0)
                msg.vy = float(0)
                msg.vz = float(0)
                self.change_yaw = plus_radian(self.change_yaw, plus_yaw) 
            elif(t_yaw-self.now_yaw>=pi and t_yaw-self.now_yaw<=2*pi-4*plus_yaw):
                self.publish_offboard_control_heartbeat_signal(True)
                self.any_direction_time -= 1
                if(self.change_yaw == -1):
                    self.change_yaw = self.now_yaw
                msg.vx = float(0)
                msg.vy = float(0)
                msg.vz = float(0)
                self.change_yaw = plus_radian(self.change_yaw, -plus_yaw)
            elif(self.now_yaw-t_yaw>4*plus_yaw and self.now_yaw-t_yaw<=pi):
                self.publish_offboard_control_heartbeat_signal(True)
                self.any_direction_time -= 1
                if(self.change_yaw == -1):
                    self.change_yaw = self.now_yaw
                msg.vx = float(0)
                msg.vy = float(0)
                msg.vz = float(0)
                self.change_yaw = plus_radian(self.change_yaw, -plus_yaw)
            elif(self.now_yaw-t_yaw>pi and self.now_yaw-t_yaw<=2*pi-4*plus_yaw):
                self.publish_offboard_control_heartbeat_signal(True)
                self.any_direction_time -= 1
                if(self.change_yaw == -1):
                    self.change_yaw = self.now_yaw
                msg.vx = float(0)
                msg.vy = float(0)
                msg.vz = float(0)
                self.change_yaw = plus_radian(self.change_yaw, plus_yaw)
            else:
                self.change_yaw = t_yaw
                self.publish_offboard_control_heartbeat_signal(False)
                msg.x = math.nan
                msg.y = math.nan
                msg.z = math.nan
                
            msg.yaw = float(self.change_yaw) 
        
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.trajectory_setpoint_publisher.publish(msg)
        if(self.offboard_setpoint_counter %5 ==0):
            self.get_logger().info(f" vx, vy, vz, yaw oper : {msg.vx, msg.vy, msg.vz, msg.yaw} \n")



    
    def circle_by_vel(self):
        self.publish_offboard_control_heartbeat_signal(False)
        if(self.initial_theta3 == -10):
            self.initial_theta3 = (self.now_yaw-math.pi/2)
            self.theta = self.initial_theta3
        msg = TrajectorySetpoint()
        w = 0.2
        msg.x = math.nan
        msg.y = math.nan
        msg.z = math.nan
        if(self.vehicle_odom.z < self.real_ground_z-3):
            if(self.vehicle_odom.z <= self.real_ground_z-4):
                msg.vz = 0.5*abs(self.real_ground_z-4-self.vehicle_odom.z)
            else :
                msg.vz = -0.5*abs(self.real_ground_z-4-self.vehicle_odom.z)
        msg.vz = float(0)
        radius = 4
        msg.vx = 0.6* float(np.cos(self.theta))
        msg.vy = 0.6* float(np.sin(self.theta))
        msg.yaw = (math.atan2(msg.vy, msg.vx)+math.pi/2) # (90 degree)
        self.theta = self.theta + w * self.dt
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
    
    def warigari_one_direction(self):
        msg = TrajectorySetpoint()
        #self.publish_offboard_control_heartbeat_signal(True)

        if self.w_count == 0: ## (0,0,z)로 이륙
            self.move_only_one_direction('z', 0.7, 200)
            if self.is_departed == 1:
                self.w_count = 1
                self.is_departed = 0
                self.one_direction_time = 0
                self.change_yaw = -1
        elif self.w_count == 1: ## (x, y, z)로 이동
            self.move_any_direction(0, 0.7, 80)
            if self.is_departed == 1:
                self.w_count = 2
                self.is_departed = 0
                self.any_direction_time = 0
                self.change_yaw = -1
        elif self.w_count == 2: 
            self.circle_by_vel()
            if (self.theta-self.initial_theta3) > math.pi*2:
                self.w_count = 3
                self.inital_theat3 = -10
                self.is_departed = 0
                self.one_direction_time = 0
                self.change_yaw = -1
        elif self.w_count == 3: ## (x, y, z)로 이동
            self.move_any_direction(0, 0.7, 30)
            if self.is_departed == 1:
                self.w_count = 4
                self.is_departed = 0
                self.one_direction_time = 0
                self.change_yaw = -1
        elif self.w_count == 4: ## (x, y, z)로 이동
            self.move_only_one_direction('n', 0.7, 200)
            if self.is_departed == 1:
                self.w_count = 5
                self.is_departed = 0
                self.one_direction_time = 0
                self.change_yaw = -1
        elif self.w_count == 5: ## (x, y, z)로 이동
            self.move_any_direction(180, 0.7,110)
            if self.is_departed == 1:
                self.w_count = 6
                self.is_departed = 0
                self.any_direction_time = 0
                self.change_yaw = -1
        elif self.w_count == 6: ## (x, y, z)로 이동
            self.move_only_one_direction('n', 0.7, 20)
            if self.is_departed == 1:
                self.w_count = 7
                self.is_departed = 0
                self.one_direction_time = 0
                self.change_yaw = -1
        
        elif self.w_count == 7: ## (0, 0, 0)로 이동 / 착륙 시 속도 고정 (0.5m/s) # ~.~ 여기 주석처리 해놨던데 살리는게 나은 것 같..음. 콜백말고 여기다 넣는게 더 깔끔하지 않아? 콜백 하나에 명령 하나 원칙도 지키기 쉽고 
            if ((-0.3+self.real_ground_z) < self.vehicle_odom.z < (0.3+self.real_ground_z) ):
                self.end_p += 1
                if(self.end_p>100):   ## ~.~ disarm 바로하니까 튕기는 이슈가 발생해서 너가 호버링 할때 했듯이 end_p로 좀 랜딩 기다렸다가 disarm 시켰어
                    self.get_logger().info(f" disarm ")
                    self.disarm()                         
                else :
                    self.get_logger().info(f" land ")
                    self.land()
            elif (- 0.5+self.real_ground_z) < self.vehicle_odom.z < (0.5+self.real_ground_z): ## 착륙 시(이걸 다르게 알 방법이 없나??) 
                self.get_logger().info(f" land ")
                self.land() ## 착륙해라
            else :
                self.move_only_one_direction('-z', 0.5, 1000)
        

        

 ##TODO : 목표점 도달 시 몇 초간 유지?/ 처음 시작할 때 그대로 안 뜨는 이유가 뭘까/ 이동하면서 odom 보정? / heading 바꿀 때 회전이 너무 빠른가?
 ##       land > disarm 문제가 있다.. 
 # ~.~ 처음 시작할때 그대로 안 뜨는 이유 : waypoint를 이동하면서 previous_yaw가 이전 yaw값을 저장하고 그 previous_yaw값을 publish 하는게 yaw값 안바꾸게 하는 기본 원리거든? 
 # ~.~ 근데 아예 처음 시작할때는 previous_yaw가 0으로 설정되어있기 때문에 0도 각도로 회전함! 이게 이유였고
 # ~.~ 근데 이륙할때 회전하면서 이륙하는 거는 아무리 생각해도 너무 불안정할 것 같아 바꿔놨어~
 # ~.~ heading 바꿀때 회전이 너무 빠른거 교수님이 좀 전에 시뮬레이션 보면서 지적했던 문제인데 미리 발견했었군.. 할일이 늘었습니다. heading 천천히 바꾸래요 ㅠㅠ 
 # ~.~ 그리고 timer_callback 한번 돌때는 publish 명령이 한번만 가게 하는게 좋을 것 같아 
 # ~.~ land > disarm 문제는 주석처리단 여러가지를 바꿨더니 해결되긴 했는데 뭐가 문제였는지는 모르겠어    
    def timer_callback(self) -> None:
        #self.get_logger().info(f" enter timer_callback ")
        if self.offboard_setpoint_counter <= 10:
            self.ground_x += self.vehicle_odom.x
            self.ground_y += self.vehicle_odom.y
            self.ground_z += self.vehicle_odom.z
            self.ground_yaw += euler_from_quaternion(self.vehicle_odom.q[0], self.vehicle_odom.q[1], self.vehicle_odom.q[2], self.vehicle_odom.q[3])
            self.ground_initial_num += 1
            # ~.~ 요것이 초기 yaw값 받아서 처음 이륙할때 회전 안하게 할 것임
        
        
        if self.offboard_setpoint_counter == 10: ## 처음, 토픽 10까지 발행 후 이륙 
            # ~.~ 이꺼 원래 self.offboard_setpoint_counter > 10 으로 되어 있었는데, 그러면 10 이상일때 시동걸라는 publish를 계속.. 보내게 될텐데 그러면 안될 것 같아서 분리했어
            self.real_ground_x = self.ground_x/self.ground_initial_num
            self.real_ground_y = self.ground_y/self.ground_initial_num
            self.real_ground_z = self.ground_z/self.ground_initial_num
            self.real_ground_yaw = self.ground_yaw/self.ground_initial_num
            self.engage_offboard_mode()
            self.arm() ## 이륙
            
        

        self.warigari_one_direction()
     


        self.offboard_setpoint_counter += 1
        if(self.offboard_setpoint_counter % 5 == 0):
            self.print_odom()
        
      
       

       
        
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
