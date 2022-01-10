#include<iostream>
#include<cmath>
#include<vector>
#include<tuple>
#include<string>
#include<array>
#include"include/dwa.h"

constexpr int obsnum = 4;
int main(){
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
    while(finish == false){
        obsPos.clear();
        for(int i = 0; i < obsnum; ++i){
            //obs[i] = changeObsPos(obs[i]);
            obsPos.push_back(obs[i]);
        }
        dwa.runToGoal(goalPos, obsPos);
        finish = dwa.goalCheck();
    }
}