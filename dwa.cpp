#include<iostream>
#include<cmath>
#include<vector>
#include<tuple>

//x, y, th, u_v, u_th
using path_tuple = std::tuple<double, double, double, double, double>;


//myenigma参考
class DWA{
    public:
        DWA(){

        }
    private:
        //パラメータ
        double max_ang_accelation;
        double max_accelation;
        double range_ang_velo;
        double lim_min_ang_velo;
        double lim_max_ang_velo;
        double lim_min_velo;
        double lim_max_velo;

        //予測時間(s)
        double pre_time;
        double pre_step;
        //探索時の刻み幅
        double delta_velo;
        double delta_ang_velo;
        //サンプリングタイム
        double sampling_time;
        //重み付け
        double weight_angle;
        double weight_velo;
        double weight_obs;
        std::vector<double> traj_paths;
        std::vector<double> traj_opt;

        //現在の値
        double velo_now;
        double ang_velo_now;

        //あとで書き換える
        double min_ang_velo;
        double max_ang_velo;
        double min_velo;
        double max_velo;
        std::vector<double> next_x;
        std::vector<double> next_y;
        std::vector<double> next_th;

        std::vector<path_tuple> path;
        std::vector<std::vector<path_tuple>> traj_path;

        double x;
        double y;
        double th;

        void calcRangeVelos(){
            //角速度
            double range_ang_velo = sampling_time * max_ang_accelation;
            min_ang_velo = ang_velo_now - range_ang_velo;
            max_ang_velo = ang_velo_now + range_ang_velo;
            //最小値
            if(min_ang_velo < lim_min_ang_velo){
                min_ang_velo = lim_min_ang_velo;
            }
            //最大値
            if(max_ang_velo > lim_min_ang_velo){
                max_ang_velo = lim_min_ang_velo;
            }

            //速度            
            double range_velo = sampling_time * max_accelation;
            min_velo = velo_now - range_velo;
            max_velo = velo_now + range_velo;
            //最小値
            if(min_velo < lim_min_velo){
                min_velo = lim_min_velo;
            }
            //最大値
            if(max_velo > lim_max_velo){
                max_velo = lim_max_velo;
            }
            
        }
        void predictState(double _ang_velo, double _velo, double _x, double _y, double _th, double _dt, double _pre_step){
            //予想状態を作成する
            double temp_x, temp_y, temp_th;
            for(int i = 0; i < _pre_step; ++i){
                temp_x = _velo * cos(_th) * _dt + _x;
                temp_y = _velo * sin(_th) * _dt + _y;
                temp_th = _ang_velo * _dt + _th;

                next_x.push_back(temp_x);
                next_y.push_back(temp_y);
                next_th.push_back(temp_th);
            }
        }
        double makePath(){
            //角度と速度の組み合わせを全探索
            path.clear();
            for(double ang_velo = min_ang_velo; ang_velo < max_ang_velo; ang_velo += delta_ang_velo){
                for(double velo = min_velo; velo < max_velo; velo += delta_velo){
                    double temp_x, temp_y, temp_th;
                    predictState(ang_velo, velo, x, y, th, sampling_time, pre_step);
                    path.push_back(path_tuple(temp_x, temp_y, temp_th, ang_velo, velo));
                }
            }
            //時刻歴Pathを保存
            traj_path.push_back(path);
        }
        double headingAngle(){

        }
        double headingVelo(){

        }
        double calcNearestObs(){

        }
        double obstcleCheck(){

        }
        double evalPath(){

        }
};

int main(){

}