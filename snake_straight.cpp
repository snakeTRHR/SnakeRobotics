#include<iostream>
#include<cmath>
#include<vector>
#include<tuple>
#include<string>
#include<array>
#include"include/serpenoid.h"

double calSerpenVel(double _vel, double _length, double _L){
    double vel=4*_length*_vel/_L;
    return vel;
}
int main(){
    double length_one_quarter=20;
    SnakeRobot snake(length_one_quarter, M_PI/4, 0.0);

    double L=snake.calL();
    std::vector<double> orbit_x;
    std::vector<double> orbit_y;

    double v=1;
    for(int i=0; i<static_cast<int>(L); ++i){
        orbit_x.push_back(v*(i+1));
        orbit_y.push_back(0);
        double vel=calSerpenVel(v, length_one_quarter, L);
        std::cout<<vel<<std::endl;
        snake.changeVel(vel);
        snake.Update();
        plt::clf();
        plt::plot(snake.C_x, snake.C_y);
        plt::plot(orbit_x, orbit_y);
        plt::pause(0.01);
    }
    /*for(int i=0; i<250; i++){
        orbit_x.push_back(v*(i+1));
        orbit_y.push_back(0);
        double vel=calSerpenVel(v, length_one_quarter, L);
        std::cout<<"vel : "<<vel<<std::endl;
        snake.changeVel(vel);
        snake.Update();
        plt::clf();
        plt::plot(snake.C_x, snake.C_y);
        plt::plot(orbit_x, orbit_y);
        plt::legend();
        plt::pause(0.01);
    }*/
}