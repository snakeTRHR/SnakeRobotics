#include<iostream>
#include<cmath>
#include<vector>
#include<tuple>

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
    std::vector<std::tuple<double, double>> data;
    double theta=0;
    while(theta<=2*M_PI){
        double temp_data_x=100*std::cos(theta);
        double temp_data_y=100*std::sin(theta);
        data.push_back(std::make_tuple(temp_data_x, temp_data_y));
        theta+=0.001;
    }
    for(int i=0; i<data.size(); i++){
        std::tuple<double, double> temp_tuple=data[i];
        std::cout<<std::get<0>(temp_tuple)<<" "<<std::get<1>(temp_tuple)<<std::endl;
    }
    std::vector<double> curvature_yaw1;
    std::vector<double> curvature_yaw2;
    std::vector<double> curvature_yaw3;
    for(int i=0; i<data.size(); ++i){
        if(i!=0&&i!=data.size()-1){
            std::tuple<double, double> temp0=data.at(i-1);
            std::tuple<double, double> temp1=data.at(i);
            std::tuple<double, double> temp2=data.at(i+1);
            double tempx0=std::get<0>(temp0);
            double tempx1=std::get<0>(temp1);
            double tempx2=std::get<0>(temp2);
            double tempy0=std::get<1>(temp0);
            double tempy1=std::get<1>(temp1);
            double tempy2=std::get<1>(temp2);
            curvature_yaw3.push_back(calCurvantureYaw(tempx0, tempx1, tempx2, tempy0, tempy1, tempy2));
            //std::cout<<tempx0<<" "<<tempx1<<" "<<tempx2<<" "<<tempy0<<" "<<tempy1<<" "<<tempy2<<std::endl;
        }
    }
    std::cout<<"----------------------------------------------------------------"<<std::endl;
    for(int i=0; i<data.size()-2; ++i){
        std::cout<<curvature_yaw3.at(i)<<std::endl;
    }
}