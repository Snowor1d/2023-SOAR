import math
import random
from matplotlib import pyplot as plt
import time


class SE:
    def __init__(self):
        self.obstacle_list = [[2.5, -2.5, 1.5], [-2.5, 2.5, 1.5]]
        self.is_path_generated = False
    def obstacle(self, obstacle_list):
        self.obstacle_list = obstacle_list
    
    def obstacle_list(self):
        return self.obstacle_list
        

    def cost(self, now_x, now_y, t_x, t_y):
        distance_target = math.sqrt((t_x - now_x) ** 2 + (t_y - now_y) ** 2)
        collision_cost = 0
        for i in self.obstacle_list:
            shortest_obstacle = (math.sqrt((i[0] - t_x) ** 2 + (i[1] - t_y) ** 2) - i[2])
            if ((math.sqrt((i[0] - t_x) ** 2 + (i[1] - t_y) ** 2) - i[2]) ==0):
                collision_cost += 10000000
            else:                
                if(shortest_obstacle<1):
                    collision_cost += (100000 / pow((math.sqrt((i[0] - t_x) ** 2 + (i[1] - t_y) ** 2) - i[2]),6))
                elif(shortest_obstacle<1.5):
                    collision_cost += (10000 / pow((math.sqrt((i[0] - t_x) ** 2 + (i[1] - t_y) ** 2) - i[2]),1))
                elif(shortest_obstacle<2):
                    collision_cost += (1000 / pow((math.sqrt((i[0] - t_x) ** 2 + (i[1] - t_y) ** 2) - i[2]),1))
                elif(shortest_obstacle<2.5):
                    collision_cost += (100 / pow((math.sqrt((i[0] - t_x) ** 2 + (i[1] - t_y) ** 2) - i[2]),1))

                        
            #if (math.sqrt(pow(i[0]-t_x,2)+pow(i[1]-t_y,2))-i[2] > 0.5):
                #collision_cost += 100000
        return distance_target + collision_cost

    def make_random_point(self, now_x, now_y, t_x, t_y):
        safe_radius = float('inf')
        shortest_obstacle = [15000, 15000]
        for i in self.obstacle_list:
            distance_target = math.sqrt((i[0] - now_x) ** 2 + (i[1] - now_y) ** 2) - i[2]
            if distance_target < safe_radius:
                safe_radius = distance_target
                shortest_obstacle[0] = i[0]
                shortest_obstacle[1] = i[1]
        if ((math.sqrt((t_x - now_x) ** 2 + (t_y - now_y) ** 2))/2 < safe_radius):
            safe_radius = (math.sqrt((t_x - now_x) ** 2 + (t_y - now_y) ** 2))/2
        generate_list = []
        #prohibit_theta = calculate_prohibit_theta(shortest_obstacle[0], now_x, shortest_obstacle[1], now_y, 1.5)
        if safe_radius < 1:
            generate_circle_num = 4
        elif safe_radius < 1.5:
            generate_circle_num = 7
        elif safe_radius < 2.5:
            generate_circle_num = 9
        elif safe_radius < 4:
            generate_circle_num = 10
        else:
            generate_circle_num = 20

        for _ in range(generate_circle_num):
            if 10 * safe_radius < 1:
                continue
            to_r = random.randint(0, int(25 * safe_radius)) / 5.0
            if to_r > safe_radius:
                to_r = safe_radius * 0.9
            theta = math.atan2(t_y - now_y, t_x - now_x)
            theta_obstacle = math.atan2(shortest_obstacle[1]-now_y, shortest_obstacle[0]-now_x)
            #if(abs(theta-theta_obstacle)>(math.pi/6)):
             #   theta += (random.randint(0, 210) + 1) / 100.0 - math.pi / 3
            #else:
            theta += (random.randint(0, 314) + 1) / 100.0 - math.pi / 2

            generate_list.append([now_x + to_r * math.cos(theta), now_y + to_r * math.sin(theta)])

        return generate_list

    def make_path3(self, now_x, now_y, t_x, t_y, path_list, num, depth):
        path_list1 = self.make_random_point(now_x, now_y, t_x, t_y)
        path_list2 = self.make_random_point(t_x, t_y, now_x, now_y)

        if num > depth:
            return
        if min(len(path_list1), len(path_list2)) <= 1:
            return
        for i in range(min(len(path_list1), len(path_list2)) - 1):
            self.make_path3(path_list1[i][0], path_list1[i][1], path_list2[i][0], path_list2[i][1], path_list, num + 1, depth)
            if self.is_path_generated:
                path_list.insert(0, [path_list1[i][0], path_list1[i][1]])
                path_list.append([path_list2[i][0], path_list2[i][1]])
                return

        if math.sqrt((now_x - t_x) ** 2 + (now_y - t_y) ** 2) < 2:
            self.is_path_generated = True

    def make_total_path(self, now_x, now_y, t_x, t_y, path_list):
        self.make_path3(now_x, now_y, t_x, t_y, path_list, 0, 1)
        if len(path_list) == 0:
            self.make_path3(now_x, now_y, t_x, t_y, path_list, 0, 2)
        if len(path_list) == 0:
            self.make_path3(now_x, now_y, t_x, t_y, path_list, 0, 3)

        for i in path_list:
            pass
            # print("생성된 좌표 :", i[0], ",", i[1])
    def make_final_path(self, now_x, now_y, target_x, target_y):
        total_path_list = []

        for _ in range(40):
            path_list = []
            se1 = SE()
            se1.make_total_path(now_x, now_y, target_x, target_y, path_list)
            if len(path_list) != 0:
                #print("성공!")
                total_path_list.append(path_list)
            #print("실패!")
        for_cost_function = SE()

        minimum_cost = float('inf')
        cost_path = 0
        final_path = []

        for i in total_path_list:
            if len(i) <= 1:
                continue
            for j in range(len(i) - 1):
                cost_path += for_cost_function.cost(i[j][0], i[j][1], i[j + 1][0], i[j + 1][1])
            cost_path += 10*len(i)
            if (cost_path < minimum_cost):
                minimum_cost = cost_path
                final_path = i
            cost_path = 0
        delete_list = []
        for i in range(len(final_path)-1):
            if (math.sqrt(pow(final_path[i][0]-final_path[i+1][0],2)+pow(final_path[i][1]-final_path[i+1][1],2)))<1:
                delete_list.append(final_path[i])
        for j in delete_list:
            final_path.remove(j)
                

        for i in final_path:
            print("to waypoint :", i[0], ",", i[1])
        return final_path
    
se1 = SE()
now_x = 10
now_y = -10
pi = math.pi
theta = 0
start_list = []
mission_r = 8.5
while(theta<2*pi):
    start_list.append([mission_r*math.cos(theta), mission_r*math.sin(theta)])
    #start_list.append([-6.25, 6])
    theta += 2*pi*(1/20)

for start in start_list:
    now_x = start[0]
    now_y = start[1]
    t_x = 0
    t_y = 0
    start = time.time()
    final_path = se1.make_final_path(now_x,now_y, t_x, t_y)
    end = time.time()
    print("실행시간 : " + str(end-start))
    x_list = []
    y_list = []
    x_list.append(now_x)
    y_list.append(now_y)
    
    for i in final_path:
        x_list.append(i[0])
        y_list.append(i[1])
    x_list.append(t_x)
    y_list.append(t_y)
    figure, axes = plt.subplots()
    for j in se1.obstacle_list:
        draw_circle = plt.Circle((j[0], j[1]), j[2], fill=False, color = 'red')
        axes.add_artist(draw_circle)
    plt.xlim([-10,10])
    plt.ylim([-10,10])
    plt.scatter(x_list, y_list)
    plt.plot(x_list, y_list)