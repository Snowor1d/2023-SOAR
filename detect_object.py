#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import random
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import math
from sklearn.cluster import KMeans

import random
import time

# Configure QoS profile for publishing and subscribing




class detect_ladder(Node):
    def __init__(self) ->None:

        super().__init__('detect_ladder')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.grid_cells_subscriber = self.create_subscription(
            GridCells, 'map_talker/grid_cells', self.PCL_callback, qos_profile)
        self.point_list = []
        self.ladder_list_guess = []
        
    def PCL_callback(self, msg):
        for i in msg.Cells:
            distance = math.sqrt(pow(i.x*0,2, 2)+pow(i.y*0.2,2))
            if(distance>3 and distance < 14):
                self.point_list.append([i.x, i.y])
        k_means = KMeans(init="k-means++", n_clusters=2, n_init=10)
        k_means.fit(self.point_list)
        k_means_cluster_centers = k_means.cluster_centers_
        self.ladder_list_guess = k_means_cluster_centers

    def r_ladder_location(self):
        return self.ladder_list_guess      

class detect_balcony(Node):
    def __init__(self) -> None:
        super().__init__('detect_balcony')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.grid_cells_subscriber2 = self.create_subscription(
            GridCells, 'map_talker/grid_cells', self.PCL_callback2, qos_profile)
        self.balcony_list =[]
        self.balcony_confirmed_x = 0
        self.balcony_confirmed_y = 0
    def PCL_callback2(self, msg):
        for i in msg.Cells:
            self.balcony_list.append([i.x, i.y])
        for j in self.balcony_list:
            self.balcony_confirmed_x += j[0]
            self.balcony_confirmed_y += j[1]
        self.balcony_confirmed_x /= len(self.balcony_list)
        self.balcony_confirmed_y /= len(self.balcony_list)
    def r_balcony_location(self):
        return [self.balcony_confirmed_x, self.balcony_confirmed_y]
    
class detect_crossbow(Node):
    def __init__(self) -> None:
        super().__init__('detect_crossbow')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.cross_location_subscriber = self.create_subscription(
            Point, 'gazebo_s2s_result', self.cross_callback, qos_profile)
        self.cross_location = [0, 0, 0]

    def cross_callback(self, msg):
        self.cross_location = [msg.x, msg.y, msg.z]
    
    def r_cross_location(self):
        return self.cross_location
        







