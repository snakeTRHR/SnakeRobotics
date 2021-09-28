#include<iostream>
#include<cmath>
#include<limits>
#include<iomanip>
#include<vector>
#include "matplotlibcpp.h"
#include "/usr/include/eigen3/Eigen/Dense"
#include "/usr/include/eigen3/Eigen/Sparse"
#include "/usr/include/eigen3/Eigen/Core"
#include "/usr/include/eigen3/Eigen/LU"

namespace plt = matplotlibcpp;

double curvature(int input){
    return sin(input * 0.01);
}
double torsion(){
}

int main(){
    //(e_r, e_p, e_s)
    Eigen::Matrix<double, 3, 1> c_s;
    Eigen::Matrix<double, 3, 1> e_r;
    Eigen::Matrix<double, 3, 1> e_p;
    Eigen::Matrix<double, 3, 1> e_s;
    //set initial value
    c_s << 0,
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

    int n = 1000;
    std::vector<double> val_x(n), val_y(n), val_z;

    for(int s = 0; s < n; ++s){
        //solve differential equation(using diagonalization)
        //https://www.momoyama-usagi.com/entry/math-ode11
        //d/dx = Ax
        //diagonalization
        Eigen::Matrix<double, 3, 3> A;
        A <<             0, curvature(s), -1 * curvature(s),
             -curvature(s),            0,                 0,
              curvature(s),            0,                 0;
        Eigen::EigenSolver<Eigen::Matrix<double, 3, 3>> eigensolver(A);
        Eigen::Matrix<double, 3, 3> P;
        P = eigensolver.eigenvectors().real();
        eigensolver.eigenvectors();
        Eigen::Matrix<double, 3, 3> D;
        D = P.inverse() * A * P;
        //P^-1x = y
        //d/dty = Dy
        Eigen::Matrix<double, 3, 1> Y;
        Y << exp(D(0, 0)),
             exp(D(1, 0)),
             exp(D(2, 0));
        //x = Py(C(s) = X)
        Eigen::Matrix<double, 3, 1> C;
        C = P * Y;
        //display graph
        //set display value
        val_x[s] = C(0, 0);
        val_y[s] = C(1, 1);
        val_z[s] = C(2, 2);
        // Clear previous plot
		plt::clf();
		// Plot line from given x and y data. Color is selected automatically.
		plt::plot(val_x, val_y);
		// Plot a line whose name will show up as "log(x)" in the legend.
		plt::named_plot("serpenoid", val_x, val_z);
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
