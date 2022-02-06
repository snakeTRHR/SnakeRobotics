#include<cmath>
#include<vector>
#include<tuple>
#include<array>
#include<utility>
#include<iostream>
#include<queue>
#include<fstream>
#include<string>
#include<iomanip>
#include "/usr/include/eigen3/Eigen/Dense"
#include "/usr/include/eigen3/Eigen/Sparse"
#include "/usr/include/eigen3/Eigen/Core"
#include "/usr/include/eigen3/Eigen/LU"
using namespace Eigen;
using catmull_tuple = std::tuple<float, float, float>;
using route_tuple = std::tuple<float, float, float, float, float>;
using std::cout;
using std::endl;
template<typename T>
class PosData2{
    public:
        T x;
        T y;
        explicit PosData2(T _x, T _y):x(_x), y(_y){}
        PosData2 operator+(PosData2 object){
            x += object.x;
            y += object.y;
            return *this;
        }
        PosData2 operator-(PosData2 object){
            x -= object.x;
            y -= object.y;
            return *this;
        }
        PosData2 operator*(PosData2 object){
            x *= object.x;
            y *= object.y;
            return *this;
        }
        void pow(){
            x *= x;
            y *= y;
        }
};
class CatmullRomSpline{
    public:
        explicit CatmullRomSpline(std::vector<route_tuple> &&input_param, int _frequency):frequency(_frequency){
            //全ての通過点を取得
            for(auto &point : input_param){
                //速度情報を省いてPosData2型にする
                PosData2<float> point2d(std::get<0>(point), std::get<1>(point));
                transit_point.push_back(point2d);
                queue_angle.push(std::get<4>(point));
            }
        }
        void operator()(){
            //始点通過、終点通過、一般経路を分類
            for(int counter = 0; counter < transit_point.size() - 1; ++counter){
                cal_L = 0;
                auto NomalCal = [&](int n, float p0, float p1, float p2, float p3){
                    a[n] = -p0 + 3.0f * p1 - 3.0f * p2 + p3;
                    b[n] = 2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3;
                    c[n] = -p0 + p2;
                    d[n] = 2.0f * p1;
                };
                auto ExceptionCal = [&](int n, float p0, float p1, float p2){
                    if(counter == 0){
                        //First Curve
                        b[n] = p0 - 2.0f * p1 + p2;
                        c[n] = -3.0f * p0 + 4.0f * p1 - p2;
                        d[n] = 2.0f * p0;
                    }else{
                        //Last Curve
                        b[n] = p0 - 2.0f * p1 + p2;
                        c[n] = -p0 + p2;
                        d[n] = 2.0f * p1;
                    }
                };
                if(counter == 0){
                    ExceptionCal(0, transit_point.at(counter).x, transit_point.at(counter + 1).x, transit_point.at(counter + 2).x);
                    ExceptionCal(1, transit_point.at(counter).y, transit_point.at(counter + 1).y, transit_point.at(counter + 2).y);
                }else if(counter == transit_point.size() - 2){
                    ExceptionCal(0, transit_point.at(counter - 1).x, transit_point.at(counter).x, transit_point.at(counter + 1).x);
                    ExceptionCal(1, transit_point.at(counter - 1).y, transit_point.at(counter).y, transit_point.at(counter + 1).y);
                }else{
                    NomalCal(0, transit_point.at(counter - 1).x, transit_point.at(counter).x, transit_point.at(counter + 1).x, transit_point.at(counter + 2).x);
                    NomalCal(1, transit_point.at(counter - 1).y, transit_point.at(counter).y, transit_point.at(counter + 1).y, transit_point.at(counter + 2).y);
                }
                for(int i = 0; i < frequency; ++i){
                    float t = static_cast<float>(i) / frequency;
                    float temp[2];
                    for(int k = 0; k < 2; ++k){
                        if(counter == 0){
                            temp[k] = 0.5f * ((b[k] * t * t) + (c[k] * t) + d[k]);
                        }else if(counter == transit_point.size() - 2){
                            temp[k] = 0.5f * ((b[k] * t * t) + (c[k] * t) + d[k]);
                        }else{
                            temp[k] = 0.5f * ((a[k] * t * t * t) + (b[k] * t * t) + (c[k] * t) + d[k]);
                        }
                    }
                    //x, yをそれぞれ入れる
                    Vector2f Pos(temp[0], temp[1]);
                    goal_point.push_back(Pos);
                    //区間距離を計算
                    cal_L += std::sqrt(Pos(0) * Pos(0) + Pos(1) * Pos(1));
                }
                queue_L_total.push(cal_L);
            }
            std::ofstream outputFile("Route.txt");
            for(auto &point : goal_point){outputFile << std::fixed << std::setprecision(5) << point(0) << " " << point(1) << "\n";}
            outputFile.close();
        }
        Vector2f getSubPoint(){
            //区間距離、角度の取得関数
            Vector2f info(queue_L_total.front(), queue_angle.front());
            queue_L_total.pop();
            queue_angle.pop();
            return info;
        }
    private:
        std::vector<float> transit_theta;
        std::vector<Vector2f> goal_point;
        std::vector<PosData2<float>> transit_point;
        std::queue<float> queue_L_total;
        std::queue<float> queue_angle;
        int frequency;
        float a[2], b[2], c[2], d[2];
        float cal_L;
};
enum class Coat{
  red1,
  red2,
  blue1,
  blue2
};
template<Coat color>
std::vector<route_tuple> routeInit(){
  std::vector<route_tuple> point;
  switch(color){
    case Coat::red1:
      point.push_back(route_tuple( 0.0f,  0.0f,  1.0f,   0.0f, 0.0f));
      point.push_back(route_tuple( 10.0f, 20.0f,  1.0f,  30.0f, 0.0f));
      point.push_back(route_tuple( 15.0f, 25.0f,  1.0f,  40.0f, 0.0f));
      point.push_back(route_tuple( 20.0f, 60.0f,  1.0f,  50.0f, 0.0f));
      point.push_back(route_tuple( 25.5f, 40.0f, -1.0f,  60.0f, 0.0f));
      point.push_back(route_tuple( 30.0f, 25.0f, -1.0f,  70.0f, 0.0f));
      point.push_back(route_tuple( 20.0f, 50.0f,  1.0f,  80.0f, 0.0f));
      point.push_back(route_tuple( 10.0f, 55.0f,  1.0f,  90.0f, 0.0f));
      point.push_back(route_tuple( 5.0f, 20.0f,  1.0f, 100.0f, 0.0f));
      point.push_back(route_tuple( 0.0f,  40.0f,  1.0f, 110.0f, 0.0f));
      break;
    case Coat::red2:
      point.push_back(route_tuple( 0.0f,  0.0f,  1.0f,   0.0f, 0.0f));
      point.push_back(route_tuple( 0.0f, 10.0f,  1.0f,  30.0f, 0.0f));
      point.push_back(route_tuple( 0.0f, 20.0f,  1.0f,  40.0f, 0.0f));
      point.push_back(route_tuple( 0.0f, 30.0f,  1.0f,  50.0f, 0.0f));
      point.push_back(route_tuple( 1.5f, 35.0f, -1.0f,  60.0f, 0.0f));
      point.push_back(route_tuple( 5.0f, 39.0f, -1.0f,  70.0f, 0.0f));
      point.push_back(route_tuple(10.0f, 40.0f,  1.0f,  80.0f, 0.0f));
      point.push_back(route_tuple(20.0f, 40.0f,  1.0f,  90.0f, 0.0f));
      point.push_back(route_tuple(30.0f, 40.0f,  1.0f, 100.0f, 0.0f));
      point.push_back(route_tuple(40.0f, 40.0f,  1.0f, 110.0f, 0.0f));
      break;
    case Coat::blue1:
      point.push_back(route_tuple( 0.0f,  0.0f,  1.0f,   0.0f, 0.0f));
      point.push_back(route_tuple( 0.0f, 10.0f,  1.0f,  30.0f, 0.0f));
      point.push_back(route_tuple( 0.0f, 20.0f,  1.0f,  40.0f, 0.0f));
      point.push_back(route_tuple( 0.0f, 30.0f,  1.0f,  50.0f, 0.0f));
      point.push_back(route_tuple( 1.5f, 35.0f, -1.0f,  60.0f, 0.0f));
      point.push_back(route_tuple( 5.0f, 39.0f, -1.0f,  70.0f, 0.0f));
      point.push_back(route_tuple(10.0f, 40.0f,  1.0f,  80.0f, 0.0f));
      point.push_back(route_tuple(20.0f, 40.0f,  1.0f,  90.0f, 0.0f));
      point.push_back(route_tuple(30.0f, 40.0f,  1.0f, 100.0f, 0.0f));
      point.push_back(route_tuple(40.0f, 40.0f,  1.0f, 110.0f, 0.0f));
      break;
    case Coat::blue2:
      point.push_back(route_tuple( 0.0f,  0.0f,  1.0f,   0.0f, 0.0f));
      point.push_back(route_tuple( 0.0f, 10.0f,  1.0f,  30.0f, 0.0f));
      point.push_back(route_tuple( 0.0f, 20.0f,  1.0f,  40.0f, 0.0f));
      point.push_back(route_tuple( 0.0f, 30.0f,  1.0f,  50.0f, 0.0f));
      point.push_back(route_tuple( 1.5f, 35.0f, -1.0f,  60.0f, 0.0f));
      point.push_back(route_tuple( 5.0f, 39.0f, -1.0f,  70.0f, 0.0f));
      point.push_back(route_tuple(10.0f, 40.0f,  1.0f,  80.0f, 0.0f));
      point.push_back(route_tuple(20.0f, 40.0f,  1.0f,  90.0f, 0.0f));
      point.push_back(route_tuple(30.0f, 40.0f,  1.0f, 100.0f, 0.0f));
      point.push_back(route_tuple(40.0f, 40.0f,  1.0f, 110.0f, 0.0f));
      break;
  }
  return point;
}

int main(){
    CatmullRomSpline splineObject(routeInit<Coat::red1>(), 100);
    splineObject();
}