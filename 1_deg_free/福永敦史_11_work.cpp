#include <stdio.h>
#include <iostream>
#include <fstream>

// パラメーター
// doubleは倍精度浮動小数点型で、非常に大きな数や小さな数を表現するのに適したデータ型
const double theta_d = 1.0;  // 目標角度 (rad)
const double I = 2.0;       // 慣性モーメント (kg.m^2) 回転のしにくさ
const double D = 3.0;       // 粘性係数 (kg.m^2/s)
const double h = 0.001;     // 刻み幅 (s)
const double kp = 100.0;    // 比例ゲイン (N.m/rad)　PID制御のP比例の要素
const double kd = 20.0;     // 微分ゲイン (N.m.s/rad) PID制御のDの微分の要素
const double t_max = 5.0;   // シミュレーション時間 (s)

// 角度制御器（PD制御）
// 目標角度と現在の角度と角速度から、制御に必要なトルクを計算する関数
double control_law(double theta, double omega) {
    // トルク = 比例ゲイン * (目標角度 - 現在の角度) - 微分ゲイン * 現在の角速度
    return kp * (theta_d - theta) - kd * omega;
}

// ロボットアームの角速度と角加速度を計算する関数
// y[0] = 角度、y[1] = 角速度
// dy[0] = 角度の時間微分(角速度)
// dy[1] = 角速度の時間微分(角加速度)
void derivative(double y[], double dy[], double torque) {
    dy[0] = y[1];
    dy[1] = torque / I - (D * y[1]) / I;
}

// ルンゲクッタ4次法
void runge_kutta(double y[], double h) {
    double k1[2], k2[2], k3[2], k4[2], y_temp[2];   // k1~k4は角速度と角加速度、y_tempはyの一時的な値
    double torque = control_law(y[0], y[1]);
    derivative(y, k1, torque);                      // その時点のロボットアームをそのトルクで動かすときの角速度と角加速度をk1に代入

    // h / 2 は、各ステップの始点、中間点、終点での傾きを計算し、これらの平均を取ることで次の点の値をより正確に推定するため
    y_temp[0] = y[0] + k1[0] * h / 2;               // 角度 = 角度 + 角速度 * 刻み幅/2
    y_temp[1] = y[1] + k1[1] * h / 2;               // 角速度 = 角速度 + 角加速度 * 刻み幅/2
    torque = control_law(y_temp[0], y_temp[1]);
    derivative(y_temp, k2, torque);                 // 始点での傾きをk2に代入

    y_temp[0] = y[0] + k2[0] * h / 2;
    y_temp[1] = y[1] + k2[1] * h / 2;
    torque = control_law(y_temp[0], y_temp[1]);
    derivative(y_temp, k3, torque);                 // 中間点での傾きをk3に代入

    y_temp[0] = y[0] + k3[0] * h;
    y_temp[1] = y[1] + k3[1] * h;
    torque = control_law(y_temp[0], y_temp[1]);
    derivative(y_temp, k4, torque);                 // 終点での傾きをk4に代入

    // 平均を取る
    y[0] += (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]) * h / 6;
    y[1] += (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]) * h / 6;
}

int main() {
    // ファイルを開く
    std::ofstream data("data.txt");

    double y[2] = {0, 0};  // y[0] = 角度、y[1] = 角速度
    for (double t = 0; t <= t_max; t += h) {
        printf("Time: %.3f, Theta: %.5f\n", t, y[0]);
        // グラフ描画
        data << t << " " << y[0] << std::endl; // Write the data to the file
        // ランゲクッタ法によるシミュレーション
        runge_kutta(y, h);
    }
    data.close();

    // グラフ描画
    FILE *gp = popen("gnuplot -persist", "w");
    if (gp == NULL) {
        std::cerr << "Cannot open pipe to gnuplot" << std::endl;
        return 1;
    }

    // fprintfはファイルに書き込む関数
    fprintf(gp, "plot 'data.txt' with lines\n");
    fflush(gp);

    pclose(gp);
    return 0;
}
