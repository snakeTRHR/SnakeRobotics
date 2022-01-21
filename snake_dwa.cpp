#include<iostream>
#include<cmath>
#include<vector>
#include<tuple>
#include<string>
#include<array>
#include"include/serpenoid.h"
#include"include/dwa.h"

constexpr int obsnum=4;

double calSerpenVel(double _vel, double _length, double _L){
    double vel=4*_length*_vel/_L;
    return vel;
}
double calCurvantureYaw(double x0, double x1, double x2, double y0, double y1, double y2){
    double dxn=x1-x0;
    double dxp=x2-x1;
    double dyn=y1-y0;
    double dyp=y2-y1;
    double dn=std::sqrt(dxn*dxn+dyn*dyn);
    double dp=std::sqrt(dxp*dxp+dyp*dyp);
    double dx=1.0/(dn+dp)*(dp/dn*dxn+dn/dp*dxp);
    double ddx=2.0/(dn+dp)*(dxp/dp-dxn/dn);
    double dy=1.0/(dn+dp)*(dp/dn*dyn+dn/dp*dyp);
    double ddy=2.0/(dn+dp)*(dyp/dp-dyn/dn);
    double curvature_yaw=(ddy*dx-ddx*dy)/(std::pow((dx*dx+dy*dy), 1.5));
    return curvature_yaw;
}
int main(){
    double length_one_quarter=20;
    SnakeRobot snake(length_one_quarter);

    std::vector<ObsSet> obsPos;
    ObsSet obs[obsnum]{ObsSet(8.5, 7.5, 0.25),
                       ObsSet(6.5, 6.0, 0.25),
                       ObsSet(4.0, 3.0, 0.25),
                       ObsSet(9.0, 4.0, 0.25)};
    for(int i=0; i<obsnum; ++i){
        obsPos.push_back(obs[i]);
    }

    GoalSet goalPos(10, 10);

    DWA dwa(goalPos);
    bool finish=false;
    double L=snake.calL();
    std::vector<double> robot_x;
    std::vector<double> robot_y;
    std::vector<double> curvature_yaw;
    double count=0;
    std::cout<<L<<std::endl;

    std::vector<std::tuple<double, double>> data;
    double theta=0;
    while(theta<=2*M_PI){
        double temp_data_x=std::cos(theta);
        double temp_data_y=std::sin(theta);
        data.push_back(std::make_tuple(temp_data_x, temp_data_y));
        theta+=0.05;
    }
    double v=5;
    for(int i=2; i<data.size(); ++i){
        /*std::tuple<double, double> temp0=data.at(i-2);
        std::tuple<double, double> temp1=data.at(i-1);
        std::tuple<double, double> temp2=data.at(i);
        double tempx0=std::get<0>(temp0);
        double tempx1=std::get<0>(temp1);
        double tempx2=std::get<0>(temp2);
        double tempy0=std::get<1>(temp0);
        double tempy1=std::get<1>(temp1);
        double tempy2=std::get<1>(temp2);
        curvature_yaw.push_back(calCurvantureYaw(tempx0, tempx1, tempx2, tempy0, tempy1, tempy2));
        //snake.changeBiasYaw(curvature_yaw.back());
        snake.changeVel(calSerpenVel(v, length_one_quarter, L));
        snake.Update();
        snake.Animation();
        std::cout<<data.size()<<" "<<i<<std::endl;*/
        snake.Update();
        while(1){
            snake.Animation();
        }
    }

    /*while(finish == false){
        ++count;
        obsPos.clear();
        for(int i=0; i<obsnum; ++i){
            //obs[i] = changeObsPos(obs[i]);
            obsPos.push_back(obs[i]);
        }
        dwa.runToGoal(goalPos, obsPos);
        finish=dwa.goalCheck();
        robot_x.push_back(dwa.getPositionX());
        robot_y.push_back(dwa.getPositionY());
        if(count>=2){
            double tempx0=robot_x[robot_x.size()-3];
            double tempx1=robot_x[robot_x.size()-2];
            double tempx2=robot_x[robot_x.size()-1];
            double tempy0=robot_y[robot_y.size()-3];
            double tempy1=robot_y[robot_y.size()-2];
            double tempy2=robot_y[robot_y.size()-1];
            curvature_yaw.push_back(calCurvantureYaw(tempx0, tempx1, tempx2, tempy0, tempy1, tempy2));
            double dwa_vel = dwa.robot_u_v;
            double dwa_ang_velo = dwa.robot_u_th;
            snake.changeBiasYaw(curvature_yaw.back());
            snake.changeVel(calSerpenVel(dwa_vel, length_one_quarter, L));
            snake.Update();
            snake.Animation();
        }
    }*/
    for(int i=0; i<curvature_yaw.size(); ++i){
        std::cout<<curvature_yaw[i]<<std::endl;
    }
}