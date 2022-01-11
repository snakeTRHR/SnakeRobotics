#include<iostream>
#include<cmath>
#include<vector>
#include<tuple>
#include<string>
#include<array>
#include"include/serpenoid.h"
#include"include/dwa.h"

constexpr int obsnum = 4;

double calCurvanture(double _vel, double _ang_vel){
    double curvanture = _ang_vel / _vel;
    return curvanture;
}
double calSerpenVel(double _vel, double _length, double _L){
    double vel = 4 * _length * _vel / _L;
    return vel;
}
int main(){
    double length_one_quarter = 20;
    SnakeRobot snake(length_one_quarter);

    std::vector<ObsSet> obsPos;
    ObsSet obs[obsnum]{ObsSet(8.5, 7.5, 0.25),
                       ObsSet(6.5, 6.0, 0.25),
                       ObsSet(4.0, 3.0, 0.25),
                       ObsSet(9.0, 4.0, 0.25)};
    for(int i = 0; i < obsnum; ++i){
        obsPos.push_back(obs[i]);
    }

    GoalSet goalPos(10, 10);

    DWA dwa(goalPos);
    bool finish = false;
    double L = snake.calL();
    while(finish == false){
        obsPos.clear();
        for(int i = 0; i < obsnum; ++i){
            //obs[i] = changeObsPos(obs[i]);
            obsPos.push_back(obs[i]);
        }
        dwa.runToGoal(goalPos, obsPos);
        finish = dwa.goalCheck();
        double dwa_vel = dwa.robot_u_v;
        double dwa_ang_velo = dwa.robot_u_th;
        snake.changeBiasYaw(calCurvanture(dwa_vel, dwa_ang_velo));
        snake.changeVel(calSerpenVel(dwa_vel, length_one_quarter, L));
        snake.Update();
        snake.Animation();
    }
}