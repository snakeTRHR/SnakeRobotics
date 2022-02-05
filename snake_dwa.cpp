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
double calSerpenBiasYaw(double _L, double _length_one_quarter, double _bias_yaw_center){
    double serpen_bias_yaw=_bias_yaw_center*_L/(4*_length_one_quarter);
    return serpen_bias_yaw;
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
double calCurvantureYaw2(double x0, double x1, double x2, double y0, double y1, double y2){
    double dx=x2-x1;
    double dy=y2-y1;
    double prev_dx=x1-x0;
    double prev_dy=y1-y0;
    double curvature=std::atan(dy/dx)-std::atan(prev_dy/prev_dx);
    return curvature;
}
int main(){
    std::vector<ObsSet> obsPos;
    //障害物位置情報
    ObsSet obs[obsnum]{ObsSet(140, 140, 2),
                       ObsSet(125, 140, 2),
                       ObsSet(130, 130, 2),
                       ObsSet(130, 135, 2)};
    for(int i=0; i<obsnum; ++i){
        obsPos.push_back(obs[i]);
    }

    GoalSet goalPos(100, 100);

    DWA dwa(goalPos);
    bool finish=false;
    std::vector<double> robot_x;
    std::vector<double> robot_y;
    std::vector<double> curvature_yaw;

    //DWAでの障害物設定
    for(int i=0; i<3; ++i){
        obsPos.clear();
        for(int i=0; i<obsnum; ++i){
            //obs[i] = changeObsPos(obs[i]);
            obsPos.push_back(obs[i]);
        }
        dwa.runToGoal(goalPos, obsPos);
        robot_x.push_back(dwa.getPositionX());
        robot_y.push_back(dwa.getPositionY());
    }

    double ini_theta=atan2(robot_y.at(1)-robot_y.at(0), robot_x.at(1)-robot_x.at(0));
    double length_one_quarter=5;
    SnakeRobot snake(length_one_quarter, ini_theta+M_PI/4, 0.0);
    double L=17.0325;
    std::vector<double> orbit_x;
    std::vector<double> orbit_y;
    int count=0;
    double prev_robot_x=0;
    double prev_robot_y=0;
    while(finish == false){
        ++count;
        obsPos.clear();
        for(int i=0; i<obsnum; ++i){
            //obs[i] = changeObsPos(obs[i]);
            obsPos.push_back(obs[i]);
        }
        dwa.runToGoal(goalPos, obsPos);
        //dwa.Animation(dwa.getPositionX(), dwa.getPositionY(), obsPos);
        finish=dwa.goalCheck();
        robot_x.push_back(dwa.getPositionX());
        robot_y.push_back(dwa.getPositionY());
        double diff_x=dwa.getPositionX()-prev_robot_x;
        double diff_y=dwa.getPositionY()-prev_robot_y;
        std::cout<<count<<" "<<dwa.robot_u_v<<" "<<std::sqrt(diff_x*diff_x+diff_y*diff_y)<<std::endl;
        prev_robot_x=dwa.getPositionX();
        prev_robot_y=dwa.getPositionY();
       
        //calc curvature
        double tempx0=robot_x[robot_x.size()-3];
        double tempx1=robot_x[robot_x.size()-2];
        double tempx2=robot_x[robot_x.size()-1];
        double tempy0=robot_y[robot_y.size()-3];
        double tempy1=robot_y[robot_y.size()-2];
        double tempy2=robot_y[robot_y.size()-1];
        //curvature_yaw.push_back(calCurvantureYaw(tempx0, tempx1, tempx2, tempy0, tempy1, tempy2));
        curvature_yaw.push_back(calCurvantureYaw2(tempx0, tempx1, tempx2, tempy0, tempy1, tempy2));
        //double dwa_vel=dwa.robot_u_v;
        double dwa_vel=std::sqrt(diff_x*diff_x+diff_y*diff_y);
        double dwa_ang_velo=dwa.robot_u_th;
        //std::cout<<robot_x.back()<<" "<<robot_y.back()<<std::endl;
        snake.changeBiasYaw(-1*calSerpenBiasYaw(L, length_one_quarter, curvature_yaw.back()));
        double serpen_vel=calSerpenVel(dwa_vel, length_one_quarter, L);
        //std::cout<<dwa_vel<<" "<<serpen_vel<<std::endl;
        //std::cout<<count<<" "<<dwa_vel<<" "<<robot_x.back()<<" "<<robot_y.back()<<std::endl;
        snake.changeVel(serpen_vel);
        snake.changeAlphaYaw(M_PI/4);
        snake.Update();
        plt::clf();
        plt::plot(snake.C_x, snake.C_y);
        plt::plot(robot_x, robot_y);
        plt::legend();
        plt::pause(0.01);
    }
    for(int i=0; i<curvature_yaw.size(); ++i){
        std::cout<<curvature_yaw[i]<<std::endl;
    }
}