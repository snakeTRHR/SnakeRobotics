#include<iostream>
#include<cmath>
#include<limits>
#include<iomanip>
#include<vector>
#include<chrono>
#include<fstream>
#include<iomanip>
#include "/usr/include/eigen3/Eigen/Dense"
#include "/usr/include/eigen3/Eigen/Sparse"
#include "/usr/include/eigen3/Eigen/Core"
#include "/usr/include/eigen3/Eigen/LU"
#include "matplotlibcpp.h"
#include"joystick.h"

using namespace std::chrono;
namespace plt = matplotlibcpp;

class SnakeRobot{
    public:
        SnakeRobot(double _length):length(_length){
            C << 0, 0, 0;
            Identity_Matrix << 1,  0,  0,
                               0, -1,  0,
                               0,  0, -1;
            Eigen::Matrix<double, 3, 3> Rotation_Matrix;
            //回転行列(固定された軸回り(三次元運動するなら任意の軸回りに変える必要ありそう))
            Rotation_Matrix <<  cos(theta_pitch) * cos(theta_yaw) + sin(theta_pitch) * sin(theta_roll) * sin(theta_yaw), -cos(theta_pitch) * sin(theta_yaw) + sin(theta_pitch) * sin(theta_roll) * cos(theta_yaw),  sin(theta_pitch) * cos(theta_roll),
                                cos(theta_roll)  * sin(theta_yaw)                                                      ,  cos(theta_roll)  * cos(theta_yaw)                                                      , -sin(theta_roll)                   ,
                               -sin(theta_pitch) * cos(theta_yaw) + cos(theta_pitch) * sin(theta_roll) * sin(theta_yaw),  sin(theta_pitch) * sin(theta_yaw) + cos(theta_pitch) * sin(theta_roll) * cos(theta_yaw),  cos(theta_pitch) * cos(theta_roll);
            Initial_Matrix = Rotation_Matrix * Identity_Matrix;
            Initial_Matrix.transposeInPlace();
            E_1 = Initial_Matrix.row(0);
            E_2 = Initial_Matrix.row(1);
            E_3 = Initial_Matrix.row(2);
        }  
        void Update(){
            SolveDE();
        }
        void Animation(){
            //matplotlibで表示
            plt::clf();
            plt::plot(C_x, C_y);
            plt::named_plot("snake", C_x, C_y);
            plt::xlim(0, 300);
            plt::ylim(-10, 10);
            plt::legend();
            plt::pause(0.01);
        }
        void Clear(){
            C_x.clear();
            C_y.clear();
            C_z.clear();
        }
    private:
        Eigen::Matrix<double, 3, 1> C;
        Eigen::Matrix<double, 3, 1> E_1;
        Eigen::Matrix<double, 3, 1> E_2;
        Eigen::Matrix<double, 3, 1> E_3;
        Eigen::Matrix<double, 3, 3> Initial_Matrix;
        Eigen::Matrix<double, 3, 3> Identity_Matrix;
        double length;
        double alpha_yaw = M_PI / 4;
        double alpha_pitch = 0;
        double bias_yaw = 0;
        double bias_pitch = 0;
        double theta_roll = 0;
        double theta_pitch = -alpha_pitch;
        double theta_yaw = alpha_yaw;
        std::vector<double> C_x, C_y, C_z;
        

            //４次ルンゲクッタで連立微分方程式を解く
            //http://skomo.o.oo7.jp/f20/hp20_4-3.htm
            Eigen::Matrix<double, 3, 1> K_a_c;
            Eigen::Matrix<double, 3, 1> K_a_1;
            Eigen::Matrix<double, 3, 1> K_a_2;
            Eigen::Matrix<double, 3, 1> K_a_3;

            Eigen::Matrix<double, 3, 1> K_b_c;
            Eigen::Matrix<double, 3, 1> K_b_1;
            Eigen::Matrix<double, 3, 1> K_b_2;
            Eigen::Matrix<double, 3, 1> K_b_3;

            Eigen::Matrix<double, 3, 1> K_c_c;
            Eigen::Matrix<double, 3, 1> K_c_1;
            Eigen::Matrix<double, 3, 1> K_c_2;
            Eigen::Matrix<double, 3, 1> K_c_3;

            Eigen::Matrix<double, 3, 1> K_d_c;
            Eigen::Matrix<double, 3, 1> K_d_1;
            Eigen::Matrix<double, 3, 1> K_d_2;
            Eigen::Matrix<double, 3, 1> K_d_3;

            double s_long = 5;
            double s = 0;
            double h = 0.05;
            double n = s_long / h;
        void SolveDE(){
            for(int i = 0; i < n; ++i){
                K_a_c = h * Func_c(s, E_1);
                K_a_1 = h * Func_1(s, E_2, E_3);
                K_a_2 = h * Func_2(s, E_1, E_3);
                K_a_3 = h * Func_3(s, E_1, E_2);

                K_b_c = h * Func_c(s + h / 2, E_1 + K_a_1 / 2);
                K_b_1 = h * Func_1(s + h / 2, E_2 + K_a_2 / 2, E_3 + K_a_3 / 2);
                K_b_2 = h * Func_2(s + h / 2, E_1 + K_a_1 / 2, E_3 + K_a_3 / 2);
                K_b_3 = h * Func_3(s + h / 2, E_1 + K_a_1 / 2, E_2 + K_a_2 / 2);


                K_c_c = h * Func_c(s + h / 2, E_1 + K_b_1 / 2);
                K_c_1 = h * Func_1(s + h / 2, E_2 + K_b_2 / 2, E_3 + K_b_3 / 2);
                K_c_2 = h * Func_2(s + h / 2, E_1 + K_b_1 / 2, E_3 + K_b_3 / 2);
                K_c_3 = h * Func_3(s + h / 2, E_1 + K_a_1 / 2, E_2 + K_b_2 / 2);

                K_d_c = h * Func_c(s + h, E_1 + K_c_1);
                K_d_1 = h * Func_1(s + h, E_2 + K_c_2, E_3 + K_c_3);
                K_d_2 = h * Func_2(s + h, E_1 + K_c_1, E_3 + K_c_3);
                K_d_3 = h * Func_3(s + h, E_1 + K_c_1, E_2 + K_c_2);

                s += h;

                C   += (K_a_c + 2 * K_b_c + 2 * K_c_c + K_d_c) / 6;
                E_1 += (K_a_1 + 2 * K_b_1 + 2 * K_c_1 + K_d_1) / 6;
                E_2 += (K_a_2 + 2 * K_b_2 + 2 * K_c_2 + K_d_2) / 6;
                E_3 += (K_a_3 + 2 * K_b_3 + 2 * K_c_3 + K_d_3) / 6;
        
                C_x.push_back(C(0, 0));
                C_y.push_back(C(1, 0));
                C_z.push_back(C(2, 0));
            }
        }
        double curvature_yaw(double _s){
            //横うねり推進(岡山大学論文参照)
            return alpha_yaw * M_PI * sin(_s * M_PI/ (2 * length))/ (2 * length) + bias_yaw;
            return 0;
        }
        double curvature_pitch(double _s){
            return 0;
        }
        double torsion(double _s){
            return 0;
        }
        Eigen::Matrix<double, 3, 1> Func_c(double _s, Eigen::Matrix<double, 3, 1> _E_1){
            return (_E_1);
        }
        Eigen::Matrix<double, 3, 1> Func_1(double _s, Eigen::Matrix<double, 3, 1> _E_2, Eigen::Matrix<double, 3, 1> _E_3){
            return (curvature_yaw(_s) * _E_2 - curvature_pitch(_s) * _E_3);
        }
        Eigen::Matrix<double, 3, 1> Func_2(double _s, Eigen::Matrix<double, 3, 1> _E_1, Eigen::Matrix<double, 3, 1> _E_3){
            return ((-1 * curvature_yaw(_s) * _E_1) + (torsion(_s) * _E_3));
        }
        Eigen::Matrix<double, 3, 1> Func_3(double _s, Eigen::Matrix<double, 3, 1> _E_1, Eigen::Matrix<double, 3, 1> _E_2){
            return (curvature_pitch(_s) * _E_1 -1 * torsion(_s) * _E_2);
        }
};
inline double get_time_sec(void){
    return static_cast<double>(duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count())/1000000000;
}
void keyboard(unsigned char key, int x, int y){
    std::cout << key << " : down" <<  std::endl;
}

void keyup(unsigned char key, int x, int y){
    std::cout << key << " : up" << std::endl;
}

void display(){}

int main(){
    double timer_start = 0;
    double timer_end = 0;
    double length_one_quarter = 8;
    SnakeRobot snake(length_one_quarter);
    timer_start = get_time_sec();
    Joystick joystick("/dev/input/js0");
    if(!joystick.isFound()){
        std::cerr << "open failed" << std::endl;
    }
    bool stop_loop = false;
    bool forth = false;
    bool back = false;
    bool right = false;
    bool left = false;
    while(!stop_loop){
        //usleep(1000);
        JoystickEvent event;
        if(joystick.sample(&event)){
            if(event.isButton() && event.number == 8 && event.value == 1){
                stop_loop = true;
                std::cout << "exit" << std::endl;
            }
            usleep(1000);
            if(event.isAxis()){
                int key_num = event.number;
                int key_val = event.value;
                switch(key_num){
                    case 0:
                    case 1:
                    case 2:
                    case 3:
                    case 4:
                    case 5:
                    case 6:
                        if(key_val > 0){
                            right = true;
                        }else if(key_val < 0){
                            left = true;
                        }else{
                            right = false;
                            left = false;
                        }
                    case 7:
                        if(key_val < 0){
                            forth = true;
                        }else{
                            forth = false;
                        }
                        break;
                    default:
                        break;

                }
            }
        }
        if(forth == true){
            snake.Update();
        }
        if(right == true){
            std::cout << "right" << std::endl;
        }
        if(left == true){
            std::cout << "left" << std::endl;
        }
        snake.Animation();
    }
}
