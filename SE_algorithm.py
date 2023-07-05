
import math
import random


class SE:
    def __init__(self):
        self.obstacle_list = [[0, 0, 1]]
        self.is_path_generated = False
    def obstacle(self, obstacle_list):
        self.obstacle_list = obstacle_list
        

    def cost(self, now_x, now_y, t_x, t_y):
        distance_target = math.sqrt((t_x - now_x) ** 2 + (t_y - now_y) ** 2)
        collision_cost = 0
        for i in self.obstacle_list:
            if ((math.sqrt((i[0] - t_x) ** 2 + (i[1] - t_y) ** 2) - i[2]) ==0):
                collision_cost += 10000000
            else:
                collision_cost += 1 / (math.sqrt((i[0] - t_x) ** 2 + (i[1] - t_y) ** 2) - i[2])
        return pow(distance_target,2) + collision_cost

    def make_random_point(self, now_x, now_y, t_x, t_y):
        safe_radius = float('inf')
        for i in self.obstacle_list:
            distance_target = math.sqrt((i[0] - now_x) ** 2 + (i[1] - now_y) ** 2) - i[2]
            if distance_target < safe_radius:
                safe_radius = distance_target
        if ((math.sqrt((t_x - now_x) ** 2 + (t_y - now_y) ** 2))/2 < safe_radius):
            safe_radius = (math.sqrt((t_x - now_x) ** 2 + (t_y - now_y) ** 2))/2
        generate_list = []
        if safe_radius < 1:
            generate_circle_num = 5
        elif safe_radius < 1.5:
            generate_circle_num = 8
        elif safe_radius < 2.5:
            generate_circle_num = 12
        elif safe_radius < 4:
            generate_circle_num = 18
        else:
            generate_circle_num = 30

        for _ in range(generate_circle_num):
            if 10 * safe_radius < 1:
                continue
            to_r = random.randint(0, int(10 * safe_radius)) / 5.0
            if to_r > safe_radius:
                to_r = safe_radius * 0.9
            theta = math.atan2(t_y - now_y, t_x - now_x)
            theta += (random.randint(0, 210) + 1) / 100.0 - math.pi / 3
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

        if math.sqrt((now_x - t_x) ** 2 + (now_y - t_y) ** 2) < 1:
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

        for _ in range(10):
            path_list = []
            se1 = SE()
            se1.make_total_path(now_x, now_y, target_x, target_y, path_list)
            if len(path_list) != 0:
                total_path_list.append(path_list)
        for_cost_function = SE()

        minimum_cost = float('inf')
        cost_path = 0
        final_path = []

        for i in total_path_list:
            if len(i) <= 1:
                continue
            for j in range(len(i) - 1):
                cost_path += for_cost_function.cost(i[j][0], i[j][1], i[j + 1][0], i[j + 1][1])
            if cost_path < minimum_cost:
                minimum_cost = cost_path
                final_path = i
            cost_path = 0

        for i in final_path:
            print("to waypoint :", i[0], ",", i[1])
        return final_path


'''
se1 = SE()
path_list = []
se1.make_final_path(0,0,0,10.5)
'''
'''
total_path_list = []
now_x = 0
now_y = 0
target_x = 0
target_y = 10.5

for _ in range(10):
    path_list = []
    se1 = SE()
    se1.make_total_path(now_x, now_y, target_x, target_y, path_list)
    if len(path_list) != 0:
        total_path_list.append(path_list)

for_cost_function = SE()

minimum_cost = float('inf')
cost_path = 0
final_path = []

for i in total_path_list:
    if len(i) <= 1:
        continue
    for j in range(len(i) - 1):
        cost_path += for_cost_function.cost(i[j][0], i[j][1], i[j + 1][0], i[j + 1][1])
    if cost_path < minimum_cost:
        minimum_cost = cost_path
        final_path = i
    cost_path = 0

print("생성된 경로 수 : " + str(len(total_path_list))+"\n")

for i in final_path:
    print("to waypoint :", i[0], ",", i[1])
        
'''
    
