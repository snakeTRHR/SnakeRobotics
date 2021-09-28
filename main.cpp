#include<iostream>
#include<cmath>
#include<vector>
#include "matplotlibcpp.h"
#include "/usr/include/eigen3/Eigen/Dense"

namespace plt = matplotlibcpp;
using Eigen::MatrixXd;

void RungeKutta(){
}

void rk4(void (*f_ptr)(),double x,double y[],double f[],int n,double h,double yw[],double k[][5])
{
    int i,j;
    double xw;
    static double c[4] = {0.0,0.5,0.5,1.0};

    for(i=0;i<4;i++){
        xw = x + c[i] * h;
        for(j=0;j<n;j++){
            yw[j] = y[j] + c[i] * k[j][i];
        }

        (*f_ptr)(xw,yw,f);

        for(j=0;j<n;j++){
            k[j][i+1] = h * f[j];
        }
    }

    for(j=0;j<n;j++){
        y[j] += (k[j][1] + 2.0*k[j][2] + 2.0*k[j][3] + k[j][4])/6.0;
    }
}

void func(double x,double y[],double f[])
{
    f[0] = y[1];
    f[1] = 1.0 - y[0] -2.0*y[1];
}

int main(){
    double a = 3;
    double b = 1;
    double kappa = 0;

    int n = 1000;
    std::vector<double> val_x(n), val_y(n), z;


    //display graph
    for(int t = 0; t < 100; ++t){
        for(int i = 0; i < 1000; ++i){
            kappa = sin(t * 0.01);
            val_x[i] = cos(kappa * i) / kappa;
            val_y[i] = sin(kappa * i) / kappa;
            std::cout << val_x[i] << " " << val_y[i] << std::endl;
        }
        // Clear previous plot
		plt::clf();
		// Plot line from given x and y data. Color is selected automatically.
		plt::plot(val_x, val_y);
		// Plot a line whose name will show up as "log(x)" in the legend.
		plt::named_plot("serpenoid", val_x, z);
		// Set x-axis to interval [0,1000000]
		plt::xlim(-10, 10);
		// Add graph title
		plt::title("Sample figure");
		// Enable legend.
		plt::legend();
		// Display plot continuously
		plt::pause(0.01);
    }

}
