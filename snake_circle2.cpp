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
    std::vector<double> robot_x;
    std::vector<double> robot_y;
    std::vector<double> curvature_yaw;
 
    std::vector<std::tuple<double, double>> data;
    double theta=0;
    constexpr double r=150;
    constexpr double dtheta=1.0/r;
    //double dtheta=2.0*M_PI/0.05;
    //double v=r*dtheta;
    double v=r*dtheta;
    int count=0;

    std::vector<double> orbit_x;
    std::vector<double> orbit_y;

    //theta=-M_PI/2;
    theta=0;
    double circumference=0;

    /*for(int i=0; i<3; ++i){
        double temp_data_x=r*std::cos(theta)-r;
        double temp_data_y=r*std::sin(theta);
        data.push_back(std::make_tuple(temp_data_x, temp_data_y));
        orbit_x.push_back(temp_data_x);
        orbit_y.push_back(temp_data_y);
        theta+=dtheta;
    }*/

    //double ini_theta=atan2(orbit_y.at(1)-orbit_y.at(0), orbit_x.at(1)-orbit_x.at(0));
    double length_one_quarter=5;
    //SnakeRobot snake(length_one_quarter, 0.0, 0.0);
    //SnakeRobot snake(length_one_quarter, ini_theta+M_PI/4, 0.0);

    std::cout<<"b"<<std::endl;
    std::ifstream file("Route.txt");
    bool flag_even=true;
    double temp_x, temp_y;
    std::cout<<"p"<<std::endl;
    while(!file.eof()){
        file>>temp_x>>temp_y;
        orbit_x.push_back(temp_x);
        orbit_y.push_back(temp_y);
        std::cout<<temp_x<<" "<<temp_y<<std::endl;
    }

    std::cout<<"k"<<std::endl;
    double ini_theta=atan2(orbit_y.at(1)-orbit_y.at(0), orbit_x.at(1)-orbit_x.at(0));
    SnakeRobot snake(length_one_quarter, ini_theta+M_PI/4, 0.0);
    
    double L=17.0325;
    count=3;
    //while(circumference<2*M_PI*r){
    while(orbit_x.size()-3){
        /*
        double temp_data_x=r*std::cos(theta)-r;
        double temp_data_y=r*std::sin(theta);
        data.push_back(std::make_tuple(temp_data_x, temp_data_y));
        orbit_x.push_back(temp_data_x);
        orbit_y.push_back(temp_data_y);
        */
        //calc curvature
        /*
        double tempx0=orbit_x[orbit_x.size()-3];
        double tempx1=orbit_x[orbit_x.size()-2];
        double tempx2=orbit_x[orbit_x.size()-1];
        double tempy0=orbit_y[orbit_y.size()-3];
        double tempy1=orbit_y[orbit_y.size()-2];
        double tempy2=orbit_y[orbit_y.size()-1];
        */
        std::cout<<"a"<<std::endl;
        double tempx0=orbit_x[count-3];
        double tempx1=orbit_x[count-2];
        double tempx2=orbit_x[count-1];
        double tempy0=orbit_y[count-3];
        double tempy1=orbit_y[count-2];
        double tempy2=orbit_y[count-1];
        double serpen_bias_yaw=calSerpenBiasYaw(L, length_one_quarter, -1*calCurvantureYaw(tempx0, tempx1, tempx2, tempy0, tempy1, tempy2));
        snake.changeBiasYaw(serpen_bias_yaw);
        double diff_x=orbit_x[count-2]-orbit_x[count-3];
        double diff_y=orbit_y[count-2]-orbit_y[count-3];
        v=std::sqrt(diff_x*diff_x+diff_y*diff_y);
        double snake_v=calSerpenVel(v, length_one_quarter, L);
        std::cout<<"snake_v="<<snake_v<<std::endl;
        snake.changeVel(snake_v);
        snake.changeAlphaYaw(M_PI/4);
        snake.Update();

        //Animation
        plt::clf();
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
    std::cout<<"result:"<<circumference<<std::endl;
  
    for(int i=0; i<curvature_yaw.size(); ++i){
        std::cout<<curvature_yaw[i]<<std::endl;
    }
}