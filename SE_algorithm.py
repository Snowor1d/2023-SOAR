#import rclpy

import math
import numpy as np
import random

class SE():

    def __init__(self):
        self.grpah = {}
        self.obstacle_list = [[0,4,1], [0,11,1]]
        self.is_path_generated = 0

    def cost(self, now_x, now_y, t_x, t_y):
        distance_target = math.sqrt(pow(t_x-now_x,2)+pow(t_y-now_y, 2))
        collision_cost = 0
        for i in self.obstacle_list:
            collision_cost += 1/(math.sqrt(pow(i[0]-t_x,2)+pow(i[1]-t_y,2))-i[2])
        return (distance_target + collision_cost)

    def make_random_point(self, now_x, now_y, t_x, t_y):
        safe_radius = math.inf
        for i in self.obstacle_list:
            distance_target = (math.sqrt(pow(i[0]-now_x,2)+pow(i[1]-now_y,2))-i[2])
            if (distance_target<safe_radius):
                safe_radius = distance_target
        #print(safe_radius)
        generate_list = []
        for j in range(0, 7):
            to_r = random.randrange(0, math.floor(5*(safe_radius)))/5
            theta = math.atan2(t_y-now_y, t_x-now_x)
            theta += (random.randrange(1, 210)/100 - math.pi/3)
            generate_list.append([now_x+to_r*np.cos(theta), now_y+to_r*np.sin(theta)])
        return generate_list


    def make_path3(self, now_x, now_y, t_x, t_y,path_list, num):
        path_list1 = self.make_random_point(now_x, now_y, t_x, t_y)
        print(path_list1[0][1])
        if (num>7):
            return
        
        for i in range(len(path_list1)):
            self.make_path3(path_list1[i][0], path_list1[i][1], t_x, t_y, path_list, num+1)
            if(self.is_path_generated == 1):
                path_list.append([now_x, now_y])
                return
        
        if(math.sqrt(pow(now_x-t_x,2)+pow(now_y-t_y,2))<1):
            print("경로 생성 완료!!\n")
            path_list.append([now_x,now_y])
            self.is_path_generated = 1


    def make_total_path(self, now_x, now_y, t_x, t_y, path_list):
        self.make_path3(now_x, now_y, t_x, t_y, path_list, 0)
        for i in path_list:
            print("생성된 좌표 : " + str(i[0]) + ", " + str(i[1]) + "\n")
        print("!!!")
        


path_list = []
se1 = SE()
se1.make_total_path(0, 0, 0, 7.5, path_list)        

        

    
