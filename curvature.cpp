#include<iostream>
#include<cmath>

double calCurvantureYaw(double x0, double x1, double x2, double y0, double y1, double y2){
    double curvature_yaw=std::atan2((y2-y1), (x2-x1))-std::atan2((y1-y0), (x1-x0));
    return curvature_yaw;
}

int main(){

}