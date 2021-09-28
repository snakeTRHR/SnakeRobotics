#include<iostream>
#include<cmath>
#include<limits>
#include<iomanip>
#include<vector>
#include "matplotlibcpp.h"
#include "/usr/include/eigen3/Eigen/Dense"
#include "/usr/include/eigen3/Eigen/Sparse"
#include "/usr/include/eigen3/Eigen/Core"

namespace plt = matplotlibcpp;

int main(){

    //(e_r, e_p, e_s)
    Eigen::Matrix<double, 3, 1> c_s;
    Eigen::Matrix<double, 3, 1> e_r;
    Eigen::Matrix<double, 3, 1> e_p;
    Eigen::Matrix<double, 3, 1> e_s;
    //set initial value
    c_s << 1,
           0,
           0;
    e_r << 1,
           0,
           0;
    e_p << 0,
           1,
           0;
    e_s << 0,
           0,
           1;

    double curvature = 0;
    double torsion = 0;

    int n = 1000;
    std::vector<double> val_x(n), val_y(n), z;

    //runge_kutta_4th_order
    //step1

    //step2

    //step3

    //step4

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
