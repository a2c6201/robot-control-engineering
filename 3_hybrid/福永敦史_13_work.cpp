#include <iostream>
#include <cmath>
#include <fstream>
#include <cstdio>

const double pi = M_PI;

// PD制御のパラメータ
const double h = 0.001; // 刻み幅(s)
const double t = 0.25;   // シミュレーション時間(s)
const double Kp = 5000; // 比例ゲイン
const double Kd = 50; // 微分ゲイン
const int N = t/h;     // シミュレーションステップ数

// ロボットアームのパラメータ
const double L1 = 0.3;
const double L2 = 0.3;

// 目標軌道
const double targets[][2] = {
	{0.3,	0.1}, // t = 0.25
	{0.5, 0.1}, // t = 0.5
	{0.4, 0.3}, // t = 0.75
	{0.3, 0.3}  // t = 1.0
};

// ロボットアーム先端から逆運動学により目標関節角度を計算する関数
void calculateAngle(double i, double xFin, double yFin, double& theta1d, double& theta2d, double& xL, double& yL) {
	// 順運動学を使用してアーム先端位置を計算する
	xL = L1 * cos(theta1d) + L2 * cos(theta1d + theta2d);
	yL = L1 * sin(theta1d) + L2 * sin(theta1d + theta2d);

	// 目標位置とアーム先端位置の誤差を計算する
	double deltaX = (xFin - xL) / (N - i);
	double deltaY = (yFin - yL) / (N - i);

	// ヤコビ行列を計算する
	double J11 = -L1 * sin(theta1d) - L2 * sin(theta1d + theta2d);
	double J12 = -L2 * sin(theta1d + theta2d);
	double J21 = L1 * cos(theta1d) + L2 * cos(theta1d + theta2d);
	double J22 = L2 * cos(theta1d + theta2d);

	// ヤコビ行列の行列式を計算する
	double detJ = J11 * J22 - J12 * J21;

	// 行列式がゼロに近い場合、逆行列が存在しないと見なし、処理を中断する
	if (fabs(detJ) < 1e-10) {
			// エラー処理などを追加するか、処理を中断するなど適切な対応を行う
			std::cerr << "Error: Jacobian matrix is singular." << std::endl;
			return;
	}

	// ヤコビ行列の逆行列を計算する
	double A11 = J22 / detJ;
	double A12 = -J12 / detJ;
	double A21 = -J21 / detJ;
	double A22 = J11 / detJ;

	// 逆ヤコビ行列を使用してtheta1とtheta2を更新する
	theta1d += A11*deltaX + A12*deltaY;
	theta2d += A21*deltaX + A22*deltaY;
}

// PD制御を使用して関節トルクを計算する関数
void calculateTorques(double theta1, double theta2, double& tau1, double& tau2, double theta1d, double theta2d, double& omega1, double& omega2) {
	omega1 = (theta1d - theta1) / h;
	omega2 = (theta2d - theta2) / h;

	tau1 = Kp * (theta1d - theta1) - Kd * (omega1);
	tau2 = Kp * (theta2d - theta2) - Kd * (omega2);
}

// PI制御を使用して押しつけ力を計算する関数
double calculateForce(double yL, double yFin, double& integralError, double Kp_force, double Ki_force) {
	double error = yFin - yL;
	integralError += error * h;
	double force = Kp_force * error + Ki_force * integralError;
	return force;
}

int main() {
	const int numPoints = sizeof(targets) / sizeof(targets[0]);
	double theta1 = pi/2.0;
	double theta2 = -pi/2.0;
	double theta1d = pi/2.0;
	double theta2d = -pi/2.0;

	// 出力ファイルの作成
	// std::ofstream outputFile("targets.txt");
	FILE *gp = popen("gnuplot", "w");
	fprintf(gp, "set terminal pdf\n");
	fprintf(gp, "set output 'plot.pdf'\n");
	fprintf(gp, "set title '2自由度系ロボットアームの位置制御'\n");
	fprintf(gp, "set xlabel 'x'\n");
	fprintf(gp, "set ylabel 'y'\n");
	fprintf(gp, "plot '-' with points pt 6 ps 0.5 title 'data'\n");

	// 目標地点ごとに計算を行う
	for (int i = 0; i < numPoints; i++) {
		double xFin = targets[i][0];
		double yFin = targets[i][1];

		// 軌道の各点までの動きを200分割し、関節角度とトルクを計算する
		for (int i = 0; i < N; i++) {
			double xL, yL;
			calculateAngle(i, xFin, yFin, theta1d, theta2d, xL, yL);

			double tau1, tau2, omega1, omega2;
			calculateTorques(theta1d, theta2d, tau1, tau2, theta1, theta2, omega1, omega2);
			// 前回の関節角度を更新する
			theta1 = theta1d;
			theta2 = theta2d;

			// PI制御による押しつけ力の計算
			double integralError = 0.0;
			double Kp_force = 1000; // 比例ゲイン
			double Ki_force = 100; // 積分ゲイン
			double force = calculateForce(yL, yFin, integralError, Kp_force, Ki_force);

			// 力制御によるトルクの補正
			tau1 += force;
			tau2 += force;

			fprintf(gp, "%f %f\n", xL, yL);
		};
	}

	fprintf(gp, "e\n");
	fflush(gp);
	std::cout << "Done!" << std::endl;
	pclose(gp);

	return 0;
}
