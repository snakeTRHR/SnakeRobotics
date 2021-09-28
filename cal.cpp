#include<stdio.h>
#include<math.h>

int main(){
    double h = 0.05;
    double euler_ans[21] = {}, heun_ans[21] = {}, runge_ans[21] = {};
    double k_1 = 0, k_2 = 0, k_3 = 0, k_4 = 0;
    euler_ans[0] = 1;
    heun_ans[0] = 1;
    runge_ans[0] = 1;

    for(int i = 1; i <= 20; ++i){
        double x = h * i;
        //オイラー
        euler_ans[i] = euler_ans[i - 1] + h * 5 * euler_ans[i - 1] / (1 + x - h);
        //ホイン
        heun_ans[i] = heun_ans[i - 1] 
                    + (h / 2) * (((5 * heun_ans[i - 1]) / (1 + x - h))
                    + ((5*(heun_ans[i - 1] + h * 5 * heun_ans[i - 1] / (1 + x - h))/(1 + x))));
        //ルンゲクッタ
        k_1 = 5 * runge_ans[i - 1] / (1 + x - h);
        k_2 = 5 * (runge_ans[i - 1] + k_1 * h / 2) / (1 + x - h + h / 2);
        k_3 = 5 * (runge_ans[i - 1] + k_2 * h / 2) / (1 + x - h + h / 2);
        k_4 = 5 * (runge_ans[i - 1] + k_2 * h) / (1 + x - h + h);
        runge_ans[i] = runge_ans[i - 1] + (h / 6) * (k_1 + 2 * k_2 + 2 * k_3 + k_4);
    }

    for(int i = 0; i <= 20; ++i){
        printf("%4.2lf %9.6lf %9.6lf %9.6lf %9.6lf\n", 
                h * i, 
                pow((1 + (h * i)), 5), 
                euler_ans[i], heun_ans[i], 
                runge_ans[i]
        );
    }
}
