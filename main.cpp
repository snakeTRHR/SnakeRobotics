#include<iostream>
#include<cmath>
#include<limits>
#include<iomanip>
#include<vector>
#include<chrono>
#include<fstream>
#include<iomanip>
#include"/usr/include/eigen3/Eigen/Dense"
#include"/usr/include/eigen3/Eigen/Sparse"
#include"/usr/include/eigen3/Eigen/Core"
#include"/usr/include/eigen3/Eigen/LU"
#include"include/matplotlibcpp.h"
#include"include/joystick.h"
#include"include/snake.h"

namespace plt = matplotlibcpp;

int main(){
    double length_one_quarter = 8;
    SnakeRobot snake(length_one_quarter);
    Joystick joystick("/dev/input/js0");
    if(!joystick.isFound()){
        std::cerr << "open failed" << std::endl;
    }
    bool stop_loop = false;
    bool forward = false;
    bool back = false;
    bool right = false;
    bool left = false;
    while(!stop_loop){
        //usleep(1000);
        JoystickEvent event;
        if(joystick.sample(&event)){
            /*if(event.isButton() && event.number == 8 && event.value == 1){
                stop_loop = true;
                std::cout << "exit" << std::endl;
            }*/
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
                            forward = true;
                        }else{
                            forward = false;
                        }
                        break;
                    default:
                        break;

                }
            }
        }
        if(forward == true){
            snake.Forward();
        }
        if(right == true){
            std::cout << "right" << std::endl;
            snake.Right();
        }
        if(left == true){
            std::cout << "left" << std::endl;
            snake.Left();
        }
        snake.Animation();
    }
}
