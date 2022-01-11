#include<iostream>
#include<cmath>
#include<limits>
#include<iomanip>
#include<vector>
#include<chrono>
#include<fstream>
#include<iomanip>
#include"/usr/include/eigen3/Eigen/Dense"
#include"/usr/include/eigen3/Eigen/Sparse"
#include"/usr/include/eigen3/Eigen/Core"
#include"/usr/include/eigen3/Eigen/LU"
#include"include/matplotlibcpp.h"
#include"include/snake.h"

namespace plt = matplotlibcpp;

int main(){
    double length_one_quarter = 80;
    SnakeRobot snake(length_one_quarter);
    for(int i = 0; i < 4*length_one_quarter; ++i){
        snake.Update();
        snake.Animation();
    }
}
