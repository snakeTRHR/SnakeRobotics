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
double calSerpenBiasYaw(double _bias_yaw, double _length, double _L){
    double bias_yaw=_L*_bias_yaw/4*_length;
    return bias_yaw;
}
int main(){
    double length_one_quarter=5;
    SnakeRobot snake(length_one_quarter, M_PI/4, 0.0);
    SnakeRobot snakeCenter(length_one_quarter, 0.0, 0.0);

    double L=snake.calL();
    std::vector<double> orbit_x;
    std::vector<double> orbit_y;

    double v=1.0;
    for(int i=0; i<static_cast<int>(100*L); ++i){
        orbit_x.push_back(v*(i+1));
        orbit_y.push_back(0);
        double vel=calSerpenVel(v, length_one_quarter, L);
        std::cout<<"vel"<<vel<<std::endl;
        //double bias_yaw_=calSerpenBiasYaw(-0.01, length_one_quarter, L);
        //std::cout<<"bias_yaw_"<<bias_yaw_<<std::endl;
        //snake.changeBiasYaw(calSerpenBiasYaw(-0.01, length_one_quarter, L));
        snake.changeBiasYaw(-0.01);
        snakeCenter.changeBiasYaw(-0.01);
        snake.changeVel(vel);
        snakeCenter.changeVel(v);
        snake.Update();
        snakeCenter.Update();
        plt::clf();
        plt::xlim(-5, 40);
        plt::ylim(-5, 40);
        plt::plot(snake.C_x, snake.C_y);
        plt::plot(snakeCenter.C_x, snakeCenter.C_y);
        //plt::plot(orbit_x, orbit_y);
        plt::pause(0.01);
    }
}