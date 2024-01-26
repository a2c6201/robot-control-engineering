#include <iostream>
#include <cmath>
#include <fstream>

double pi = M_PI;

// PD制御のパラメータ
const double h = 0.001; // 刻み幅(s)
const double t = 0.2;   // シミュレーション時間(s)
const double Kp = 1.0; // 比例ゲイン
const double Kd = 0.5; // 微分ゲイン

// ロボットアームのパラメータ
const double L1 = 0.3;
const double L2 = 0.3;

// 目標軌道
double targets[][2] = {
  {0.3, 0.3}, // t = 0
  {0.4, 0.3}, // t = 0.2
  {0.5, 0.1}, // t = 0.4
  {0.3, 0.1}, // t = 0.6
  {0.3, 0.3}  // t = 0.8
};

// ロボットアーム先端から逆運動学により目標関節角度を計算する関数
void calculateAngle(double x, double y, double& theta1, double& theta2) {
  double epsilon = 1e-6; // 許容誤差

  theta1 = pi/2.0;
  theta2 = -pi/2.0;

  // 分割数
  int N = t/h;

  // ヤコビ行列を使用して逆運動学を解く
  for (int i = 0; i < N; i++) {

    // 順運動学を使用してアーム先端位置を計算する
    double xEnd = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    double yEnd = L1 * sin(theta1) + L2 * sin(theta1 + theta2);

    // 目標位置とアーム先端位置の誤差を計算する
    double deltaX = (x-xEnd) / (N-i);
    double deltaY = (y-yEnd) / (N-i);

    // ヤコビ行列を計算する
    double J11 = -L1 * sin(theta1) - L2 * sin(theta1 + theta2);
    double J12 = -L2 * sin(theta1 + theta2);
    double J21 = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    double J22 = L2 * cos(theta1 + theta2);

    // ヤコビ行列の逆行列を計算する
    double detJ = J11 * J22 - J12 * J21;
    double A11 = J22 / detJ;
    double A12 = -J12 / detJ;
    double A21 = -J21 / detJ;
    double A22 = J11 / detJ;

    // 逆ヤコビ行列を使用してtheta1とtheta2を更新する
    theta1 += theta1 + A11*deltaX + A12*deltaY;
    theta2 += theta2 - A21*deltaX - A22*deltaY;
  }
}

// PD制御を使用して関節トルクを計算する関数
void calculateTorques(double theta1, double theta2, double& tau1, double& tau2, double prevTheta1, double prevTheta2) {
  double error1 = theta1 - prevTheta1;
  double error2 = theta2 - prevTheta2;

  tau1 = Kp * error1 - Kd * (theta1 - prevTheta1);
  tau2 = Kp * error2 - Kd * (theta2 - prevTheta2);
}

int main() {
  const int numPoints = sizeof(targets) / sizeof(targets[0]);

  // 変数の初期化
  double prevTheta1 = 0.0;
  double prevTheta2 = 0.0;

  // 出力ファイルの作成
  std::ofstream outputFile("targets.pdf");

  // 軌道の各点で関節角度とトルクを計算する
  for (int i = 0; i < numPoints; i++) {
    double x = targets[i][0];
    double y = targets[i][1];

    double theta1, theta2;
    calculateAngle(x, y, theta1, theta2);

    double tau1, tau2;
    calculateTorques(theta1, theta2, tau1, tau2, prevTheta1, prevTheta2);

    // 前回の関節角度を更新する
    prevTheta1 = theta1;
    prevTheta2 = theta2;

    // 関節角度とトルクを表示する
    std::cout << "Point " << i+1 << ": "
              << "Theta1 = " << theta1 << ", "
              << "Theta2 = " << theta2 << ", "
              << "Tau1 = " << tau1 << ", "
              << "Tau2 = " << tau2 << std::endl;

    // 軌道を出力ファイルに書き込む
    outputFile << x << " " << y << std::endl;
  }

  // 出力ファイルを閉じる
  outputFile.close();

  return 0;
}
