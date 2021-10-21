#include<iostream>
#include<cmath>

#include"matplotlibcpp.h"
namespace plt = matplotlibcpp;

int main(){
    double center_x = 1;
    double center_y = 1;
    double radius = 1;
    std::vector<double> write_x;
    std::vector<double> write_y;
    for(int k = 0; k < 100; ++k){
        double theta = 2 * M_PI * k / 100;
        write_x.push_back(center_x + radius * cos(theta));
        write_y.push_back(center_y + radius * sin(theta));
    }
    plt::plot(write_x, write_y, "b");
    plt::xlim(-1.5, 3.0);
    plt::ylim(-1.5, 3.0);
    plt::show();
}