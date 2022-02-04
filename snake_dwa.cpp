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
int main(){
    std::vector<ObsSet> obsPos;
    ObsSet obs[obsnum]{ObsSet(10, 20, 2),
                       ObsSet(25, 40, 2),
                       ObsSet(30, 30, 2),
                       ObsSet(40, 35, 2)};
    for(int i=0; i<obsnum; ++i){
        obsPos.push_back(obs[i]);
    }

    GoalSet goalPos(50, 50);

    DWA dwa(goalPos);
    bool finish=false;
    std::vector<double> robot_x;
    std::vector<double> robot_y;
    std::vector<double> curvature_yaw;
 
    /*double theta=0;
    constexpr double r=100;
    constexpr double dtheta=1.0/r;
    //double dtheta=2.0*M_PI/0.05;
    //double v=r*dtheta;
    double v=r*dtheta;
    int count=0;

    std::vector<double> orbit_x;
    std::vector<double> orbit_y;

    theta=-M_PI/2;*/
    /*
    v=L/100;
    snake.changeVel(0.8);
    for(int i=0; i<4*length_one_quarter; i++){
        orbit_x.push_back(v*(i+1));
        orbit_y.push_back(0);
        double vel=calSerpenVel(v, length_one_quarter, L);
        std::cout<<"vel : "<<vel<<std::endl;
        //snake.changeVel(vel);
        snake.changeAlphaYaw(M_PI/4);
        snake.Update();
        plt::clf();
        plt::plot(snake.C_x, snake.C_y);
        //plt::plot(orbit_x, orbit_y);
        plt::legend();
        plt::pause(0.01);
    }*/

    /*v=1;
    for(int i=0; i<static_cast<int>(L); ++i){
        orbit_x.push_back(v*(i+1));
        orbit_y.push_back(0);
        double vel=calSerpenVel(v, length_one_quarter, L);
        std::cout<<"vel"<<vel<<std::endl;
        //snake.changeAlphaYaw(M_PI/4);
        snake.changeVel(vel);
        snake.Update();
        plt::clf();
        plt::plot(snake.C_x, snake.C_y);
        plt::plot(orbit_x, orbit_y);
        plt::pause(0.01);
    }*/
    /*double circumference=0;
    while(circumference<2*M_PI*r){
        double temp_data_x=r*std::cos(theta);
        double temp_data_y=r*std::sin(theta)+r;
        data.push_back(std::make_tuple(temp_data_x, temp_data_y));
        orbit_x.push_back(temp_data_x);
        orbit_y.push_back(temp_data_y);

        double serpen_bias_yaw=calSerpenBiasYaw(L, length_one_quarter, -0.01);
        //snake.changeBiasYaw(-0.0085);
        //snake.changeBiasYaw(-0.01);
        snake.changeBiasYaw(serpen_bias_yaw);
        double snake_v=calSerpenVel(v, length_one_quarter, L);
        std::cout<<"snake_v="<<snake_v<<std::endl;
        snake.changeVel(snake_v);
        //snake.changeAlphaYaw(0);
        snake.changeAlphaYaw(M_PI/4);
        snake.Update();
        plt::clf();
        plt::xlim(-105, 105);
        plt::ylim(-5, 205);
        plt::plot(snake.C_x, snake.C_y);
        plt::plot(orbit_x, orbit_y);
        plt::legend();
        plt::pause(0.01);

        std::cout<<theta*360/M_PI<<std::endl;
        theta+=dtheta;
        ++count;
        circumference+=v;
        std::cout<<"count="<<count<<std::endl;
    }
    std::cout<<"result:"<<circumference<<std::endl;*/
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
    while(finish == false){
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
    
        /*plt::clf();
        plt::xlim(0, 55);
        plt::ylim(0, 55);
        plt::plot(robot_x, robot_y);
        plt::legend();
        plt::pause(0.01);*/

        //calc curvature
        double tempx0=robot_x[robot_x.size()-3];
        double tempx1=robot_x[robot_x.size()-2];
        double tempx2=robot_x[robot_x.size()-1];
        double tempy0=robot_y[robot_y.size()-3];
        double tempy1=robot_y[robot_y.size()-2];
        double tempy2=robot_y[robot_y.size()-1];
        curvature_yaw.push_back(calCurvantureYaw(tempx0, tempx1, tempx2, tempy0, tempy1, tempy2));
        double dwa_vel = dwa.robot_u_v;
        double dwa_ang_velo = dwa.robot_u_th;
        snake.changeBiasYaw(calSerpenBiasYaw(L, length_one_quarter, curvature_yaw.back()));
        snake.changeVel(calSerpenVel(dwa_vel, length_one_quarter, L));
        snake.changeAlphaYaw(M_PI/4);
        snake.Update();
        snake.Animation();
        
    }
    for(int i=0; i<curvature_yaw.size(); ++i){
        std::cout<<curvature_yaw[i]<<std::endl;
    }
}