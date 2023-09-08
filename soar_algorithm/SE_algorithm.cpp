#include <iostream>
#include <cmath>
#include <vector>
#include <random>
#include <iterator>




class SE {
private:
    std::vector<std::vector<double>> obstacle_list;
    std::vector<std::vector<double>> graph;
    bool is_path_generated;

public:
    SE() {
        obstacle_list = {{0, 4, 1}, {0, 11, 1}};
        is_path_generated = false;
    }

    double cost(double now_x, double now_y, double t_x, double t_y) {
        double distance_target = std::sqrt(std::pow(t_x - now_x, 2) + std::pow(t_y - now_y, 2));
        double collision_cost = 0;
        for (const auto& i : obstacle_list) {
            collision_cost += 1 / (std::sqrt(std::pow(i[0] - t_x, 2) + std::pow(i[1] - t_y, 2)) - i[2]);
        }
        return (distance_target + collision_cost);
    }

    std::vector<std::vector<double>> make_random_point(double now_x, double now_y, double t_x, double t_y) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dis(0, 100000);
        double safe_radius = std::numeric_limits<double>::infinity();
        for (const auto& i : obstacle_list) {
            double distance_target = (std::sqrt(std::pow(i[0] - now_x, 2) + std::pow(i[1] - now_y, 2)) - i[2]);
            if (distance_target < safe_radius) {
                safe_radius = distance_target;
            }
        }
        
        std::vector<std::vector<double>> generate_list;
        int generate_circle_num = 0;
        if(safe_radius<1)
            generate_circle_num = 5;
        else if(safe_radius<1.5)
            generate_circle_num = 8;
        else if(safe_radius<2.5)
            generate_circle_num = 12;
        else
            generate_circle_num = 20;
        for (int j = 0; j < generate_circle_num; j++) {
            if((10*safe_radius) < 1)
                continue;
            double to_r = static_cast<double>((dis(gen) % static_cast<int>(10 *(safe_radius)) / 5.0));
            if (to_r>safe_radius)
                to_r = safe_radius*0.9;
            double theta = std::atan2(t_y - now_y, t_x - now_x);
            theta += ((static_cast<double>(dis(gen) % 210) + 1) / 100.0 - M_PI / 3);
            generate_list.push_back({now_x + to_r * std::cos(theta), now_y + to_r * std::sin(theta)});
        }
        return generate_list;
    }
    void make_path3(double now_x, double now_y, double t_x, double t_y, std::vector<std::vector<double>>& path_list, int num) {
        std::vector<std::vector<double>> path_list1 = make_random_point(now_x, now_y, t_x, t_y);
        std::vector<std::vector<double>> path_list2 = make_random_point(t_x, t_y, now_x, now_y);
        
        if (num > 3) {
            return;
        }
        if(std::min(path_list1.size(), path_list2.size())<=1)
            return;
        for (int i=0; i< std::min(path_list1.size(), path_list2.size())-1; i++) {
            make_path3(path_list1[i][0], path_list1[i][1], path_list2[i][0], path_list2[i][1], path_list, num + 1);
            if (is_path_generated) {
                path_list.insert(path_list.begin(), {path_list1[i][0], path_list1[i][1]});
                path_list.push_back({path_list2[i][0], path_list2[i][1]});
                return;
            }
        }
        
        if (std::sqrt(std::pow(now_x - t_x, 2) + std::pow(now_y - t_y, 2)) < 1) {
            std::cout << "경로 생성 완료!!" << std::endl;
            //path_list.insert(path_list.begin(),{now_x, now_y});
            //path_list.push_back({t_x, t_y});
            is_path_generated = true;
        }
    }

    void make_total_path(double now_x, double now_y, double t_x, double t_y, std::vector<std::vector<double>>& path_list) {
        make_path3(now_x, now_y, t_x, t_y, path_list, 0);
        for (const auto& i : path_list) {
            std::cout << "생성된 좌표 : " << i[0] << ", " << i[1] << std::endl;
        }
    }
};

int main() {
    std::vector<std::vector<std::vector<double>>> total_path_list;
    int now_x;
    int now_y;
    int target_x;
    int target_y;
    now_x = 0;
    now_y = 0;
    target_x = 0;
    target_y = 7.5;
    for (int i=0; i<20; i++){
        std::vector<std::vector<double>> path_list;
        SE se1;
        se1.make_total_path(now_x, now_y, target_x, target_y, path_list);
        if(path_list.size()!=0)
            total_path_list.push_back(path_list);
    }
    SE for_cost_function;
    
    double minimum_cost = std::numeric_limits<double>::infinity();
    double cost_path = 0;
    std::vector<std::vector<double>> final_path;
    for(auto& i: total_path_list){
        if(i.size()<=1)
            continue;
        for(int j=0; j<i.size()-1; j++){
            cost_path += for_cost_function.cost(i[j][0], i[j][1], i[j+1][0], i[j+1][1]);
        }
        if (cost_path<minimum_cost){
            minimum_cost = cost_path;
            final_path = i;
        }
        cost_path = 0;
    }

    for (const auto& i : final_path) {
        std::cout << "to waypoint : " << i[0] << ", " << i[1] << std::endl;
    }


    //std::cout << total_path_list.size() << std::endl;
    return 0;
}