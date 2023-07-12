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



class detect_ladder(Node):
    def __init__(self):
         self.grid_cells_subscriber = self.create_subscription(
            GridCells, 'map_talker/grid_cells', self.PCL_callback)
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
    def __init__(self):
         self.grid_cells_subscriber = self.create_subscription(
            GridCells, 'map_talker/grid_cells', self.PCL_callback)
         self.balcony_list =[]
         self.balcony_confirmed_x = 0
         self.balcony_confirmed_y = 0
    def PCL_callback(self, msg):
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

    def __init__(self):
        self.cross_location_subscriber = self.create_subscription(
            Point, 'gazebo_s2s_result', self.cross_callback)
        self.cross_location = [0, 0, 0]

    def cross_callback(self, msg):
        self.cross_location = [msg.Point.x, msg.Point.y, msg.Point.z]
    
    def r_cross_location(self):
        return self.cross_location
        


        num_l = [[1,2], [2,3], [4,5], [2.4, 1.2], [1.2, 3.4], [100, 101]]
num = []
for k in range(100):
    if k%2==0:
        num.append([random.randrange(1,100), random.randrange(1,100)])
    else :
        num.append([random.randrange(200, 300), random.randrange(200,300)])
num = np.array(num)

k_means1 =  KMeans(init="k-means++", n_clusters=2, n_init = 6)
start = time.time()
k_means1.fit(num)
end = time.time()
k_means_cluster_centers2 = k_means1.cluster_centers_

print(k_means_cluster_centers2)
print(end-start)





