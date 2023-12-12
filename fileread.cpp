#include<iostream>
#include<fstream>
#include<vector>
#include<string>

using namespace std;

int main() {
    std::ifstream file("Route.txt");
    vector<double> data_x;
    vector<double> data_y;
    std::string line;
    bool flag_even=true;
    double temp_x, temp_y;
    while(!file.eof()){
        file>>temp_x>>temp_y;
        data_x.push_back(temp_x);
        data_y.push_back(temp_y);
    }
    for(int i=0; i<data_x.size(); ++i){
        std::cout<<data_x[i]<<" "<<data_y[i]<<std::endl;
    }
}