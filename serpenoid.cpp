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
#include"include/serpenoid.h"

namespace plt=matplotlibcpp;

int main(){
    double length_one_quarter = 5;
    SnakeRobot snake(length_one_quarter, M_PI/4, 0.0);
    double L=snake.calL();
    std::cout<<L<<std::endl;
    /*for(int i=0; i<4*length_one_quarter; ++i){
        snake.Update();
        snake.Animation();
    }*/
}
