/***************************************************************
*  本文件包含一个标准测试函数类，用来测试优化算法的性能                  *
*  @author   ZhRiT                                             *
*  @email    buaazhangrit@163.com                              *
*  @version  1.0.0                                             *
*  @date     2018-09-20                                        *
***************************************************************/
#pragma once

#include <cmath>
#include <time.h>
using namespace std;

#ifndef PI
#define PI 3.1415926
#endif

/**
 * @brief 21个标准测试函数
 */
class Benchmark {
public:
	Benchmark() {};
	virtual ~Benchmark() {};

	static int T;
	
	/**
	 * @brief No.1
	 *   -name          Sphere Model 
	 *   -formula       f = Σ{i=1,D}[x_i^2]
	 *   -Search Space  [-100, 100]^D
	 *   -f_min         f(0,...,0) = 0 
	 *   -brief         单极值 unimodal
	 * @param x 自变量
	 * @param d 维数
	 * @return 函数值
	*/
	static double Sphere(double* x, int d) {
		double s = 0;
		for (int i = 0; i < d; i++) {
			s += x[i] * x[i];
		}
		Benchmark::T++;
		return s;
	}

	/**
	 * @brief No.2
	 *   -name          Schwefel's Problem 1.2
	 *   -formula       f = Σ{i=1，D}[(Σ{j=1,i}[x_j])^2]
	 *   -Search Space  [-100, 100]^D
	 *   -f_min         f(0,...,0) = 0
	 *   -brief         单极值 unimodal
	 * @param x 自变量
	 * @param d 维数
	 * @return 函数值
	*/
	static double Schwefel1_2(double* x, int d) {
		double s = 0.0, s0 = 0.0;
		for (int i = 0; i < d; i++) {
			s0 = 0.0;
			for (int j = 0; j <= i; j++) {
				s0 += x[j];
			}
			s += s0 * s0;
		}
		Benchmark::T++;
		return s;
	}

	/**
	 * @brief No.3
	 *   -name          Schwefel's Problem 2.22
	 *   -formula       f = Σ{i=1，D}[|x_i|] + ∏{i=1,D}[|x_i|]
	 *   -Search Space  [-10, 10]^D
	 *   -f_min         f(0,...,0) = 0
	 *   -brief         单极值 unimodal
	 * @param x 自变量
	 * @param d 维数
	 * @return 函数值
	*/
	static double Schwefel2_22(double* x, int d) {
		double s = 0.0, p = 1.0;
		for (int i = 0; i < d; i++) {
			s += abs(x[i]);
			p *= abs(x[i]);
		}
		Benchmark::T++;
		return s + p;
	}

	/**
	 * @brief No.4
	 *   -name          Schwefel's Problem 2.21
	 *   -formula       f = max{|x_i|, 1<=i<=D}
	 *   -Search Space  [-100, 100]^D
	 *   -f_min         f(0,...,0) = 0
	 *   -brief         
	 * @param x 自变量
	 * @param d 维数
	 * @return 函数值
	*/
	static double Schwefel2_21(double* x, int d) {
		double f = 0.0;
		for (int i = 0; i < d; i++) {
			if (abs(x[i]) > f) {
				f = abs(x[i]);
			}
		}
		Benchmark::T++;
		return f;
	}

	/**
	 * @brief No.5
	 *   -name          Generalized Rosenbrock’s Function
	 *   -formula       f = Σ{i=1,D-1}[100(x_{i+1}-x_i^2)^2+(x_i-1)^2]
	 *   -Search Space  [-30, 30]^D
	 *   -f_min         f(1,...,1) = 0
	 *   -brief
	 * @param x 自变量
	 * @param d 维数
	 * @return 函数值
	*/
	static double Generalized(double* x, int d) {
		double s = 0.0;
		for (int i = 0; i < d - 1; i++) {
			s += 100.0 * pow(x[i + 1] - pow(x[i], 2), 2) + pow(x[i] - 1, 2);
		}
		Benchmark::T++;
		return s;
	}

	/**
	 * @brief No.6
	 *   -name          Step Function
	 *   -formula       f = Σ{i=1,D}[(floor(x_i+0.5))^2]
	 *   -Search Space  [-100, 100]^D
	 *   -f_min         f(0,...,0) = 0
	 *   -brief
	 * @param x 自变量
	 * @param d 维数
	 * @return 函数值
	*/
	static double Step(double* x, int d) {
		double s = 0.0;
		for (int i = 0; i < d; i++) {
			s += pow(floor(x[i] + 0.5), 2);
		}
		Benchmark::T++;
		return s;
	}

	/**
	 * @brief No.7
	 *   -name          Quartic Function, i.e., Noise
	 *   -formula       f = Σ{i=1,D}[i*x_i^4]+random[0,1)
	 *   -Search Space  [-1.28, 1.28]^D
	 *   -f_min         f(0,...,0) = 0
	 *   -brief         （暂时不用）
	 * @param x 自变量
	 * @param d 维数
	 * @return 函数值
	*/
	static double Quartic(double* x, int d) {
		//srand((unsigned int)time(0));
		double s = 0.0;
		for (int i = 0; i < d; i++) {
			s += (i + 1.0) * pow(x[i], 4);
		}
		s += rand() / double(RAND_MAX);
		Benchmark::T++;
		return s;
	}

	/**
	 * @brief No.8
	 *   -name          Generalized Schwefel’s Problem 2.26
	 *   -formula       f = Σ{i=1,D}[x_i*sin(sqrt(abs(x_i)))]
	 *   -Search Space  [-500, 500]^D
	 *   -f_min         f(420.9687,...,420.9687) = -418.98289*D
	 *   -brief         多极值 multimodal
	 * @param x 自变量
	 * @param d 维数
	 * @return 函数值
	*/
	static double Generalized_Schwefel2_26(double* x, int d) {
		double s = 0.0;
		for (int i = 0; i < d; i++) {
			s += x[i] * sin(sqrt(abs(x[i])));
		}
		Benchmark::T++;
		return s;
	}

	/**
	 * @brief No.9
	 *   -name          Generalized Rastrigin’s Function
	 *   -formula       f = Σ{i=1,D}[x_I^2-10cos(2PIx_i)+10]
	 *   -Search Space  [-5.12, 5.12]^D
	 *   -f_min         f(0,...,0) = 0
	 *   -brief         多极值 multimodal
	 * @param x 自变量
	 * @param d 维数
	 * @return 函数值
	*/
	static double Generalized_Rastrigin(double* x, int d) {
		double s = 0.0;
		for (int i = 0; i < d; i++) {
			s += pow(x[i], 2) - 10.0 * cos(2 * PI * x[i]) + 10.0;
		}
		Benchmark::T++;
		return s;
	}

	/**
	 * @brief No.10
	 *   -name          Ackley’s Function
	 *   -formula       f = -20exp(-0.2sqrt(1/DΣ{i=1,D}[x_i^2]))-exp(1/DΣ{i=1,D}[cos(2pix_i)])+20+exp(1)
	 *   -Search Space  [-32, 32]^D
	 *   -f_min         f(0,...,0) = 0
	 *   -brief         多极值 multimodal
	 * @param x 自变量
	 * @param d 维数
	 * @return 函数值
	*/
	static double Ackley(double* x, int d) {
		double s1 = 0.0, s2 = 0.0;
		for (int i = 0; i < d; i++) {
			s1 += x[i] * x[i];
			s2 += cos(2.0 * PI * x[i]);
		}
		Benchmark::T++;
		return -20.0 * exp(-0.2 * sqrt(1.0 / d * s1)) - exp(1.0 / d * s2) + 20.0 + exp(1.0);
	}

	/**
	 * @brief No.11
	 *   -name          Generalized Griewank Function
	 *   -formula       f = 1/4000*Σ{i=1,D}[x_i^2]-∏{i=1,D}[cos(x_i/sqrt(i)] + 1
	 *   -Search Space  [-600, 600]^D
	 *   -f_min         f(0,...,0) = 0
	 *   -brief         多极值 multimodal
	 * @param x 自变量
	 * @param d 维数
	 * @return 函数值
	*/
	static double Generalized_Griewank(double* x, int d) {
		double s = 0.0, p = 1.0;
		for (int i = 0; i < d; i++) {
			s += x[i] * x[i];
			p *= cos(x[i] / sqrt(i + 1.0));
		}
		Benchmark::T++;
		return 1.0 / 4000.0 * s - p + 1.0;
	}

	/**
	 * @brief No.12
	 *   -name          Generalized Penalized Functions 1
	 *   -formula       https://wenku.baidu.com/view/97a0880102020740be1e9b86.html No.12
	 *   -Search Space  [-50, 50]^D
	 *   -f_min         f(1,...,1) = 0
	 *   -brief         多极值 multimodal
	 * @param x 自变量
	 * @param d 维数
	 * @return 函数值
	*/
	static double Generalized_Penalized_1(double* x, int d) {
		double s1 = 0.0, s2 = 0.0;
		for (int i = 0; i < d - 1; i++) {
			s1 += pow(gety(x[i]) - 1.0, 2) * (1.0 + 10.0 * pow(sin(PI * gety(x[i + 1])), 2));
		}
		for (int i = 0; i < d; i++) {
			s2 += getu(x[i], 10.0, 100.0, 4.0);
		}
		Benchmark::T++;
		return PI / d * (10.0 * pow(sin(PI * gety(x[0])), 2) + s1 + pow(gety(x[d - 1]) - 1.0, 2)) + s2;
	}

	/**
	 * @brief No.13
	 *   -name          Generalized Penalized Functions 2
	 *   -formula       https://wenku.baidu.com/view/97a0880102020740be1e9b86.html No.12
	 *   -Search Space  [-50, 50]^D
	 *   -f_min         f(1,...,1) = 0
	 *   -brief         多极值 multimodal
	 * @param x 自变量
	 * @param d 维数
	 * @return 函数值
	*/
	static double Generalized_Penalized_2(double* x, int d) {
		double s1 = 0.0, s2 = 0.0;
		for (int i = 0; i < d - 1; i++) {
			s1 += pow(x[i] - 1.0, 2) * (1.0 + pow(sin(3 * PI * x[i + 1]), 2));
		}
		for (int i = 0; i < d; i++) {
			s2 += getu(x[i], 5.0, 100.0, 4.0);
		}
		Benchmark::T++;
		return 0.1 * (pow(sin(3 * PI * x[0]), 2) + s1 + 
			pow(x[d - 1] - 1.0, 2) * (1 + pow(sin(2 * PI * x[d - 1]), 2))) + s2;
	}

	/**
	 * @brief No.14
	 *   -name          Shekel’s Foxholes Function
	 *   -formula       https://wenku.baidu.com/view/97a0880102020740be1e9b86.html No.13
	 *   -Search Space  [-65.536, 65.536]^2
	 *   -f_min         f(-32, -32) ≈ 1
	 *   -brief         
	 * @param x 自变量
	 * @param d 维数
	 * @return 函数值
	*/
	static double Shekel_Foxholes(double* x, int d = 2) {
		double a[2][25] = { {-32.0, -16.0, 0.0, 16.0, 32.0, -32.0, -16.0, 0.0, 16.0, 32.0,
			-32.0, -16.0, 0.0, 16.0, 32.0, -32.0, -16.0, 0.0, 16.0, 32.0,
			-32.0, -16.0, 0.0, 16.0, 32.0},{-32.0, -32.0, -32.0, -32.0, -32.0,
			-16.0, -16.0, -16.0, -16.0, -16.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			16.0, 16.0, 16.0, 16.0, 16.0, 32.0, 32.0, 32.0, 32.0, 32.0, } };
		double s = 0.0, s0 = 0.0;
		for (int j = 0; j < 25; j++) {
			s0 = 0.0;
			for (int i = 0; i < 2; i++) {
				s0 += pow(x[i] - a[i][j], 6);
			}
			s += 1.0 / (j + 1.0 + s0);
		}
		s += 1.0 / 500.0;
		Benchmark::T++;
		return 1.0 / s;
	}

	/**
	 * @brief No.15
	 *   -name          Kowalik’s Function
	 *   -formula       https://wenku.baidu.com/view/97a0880102020740be1e9b86.html No.14
	 *   -Search Space  [-5, 5]^4
	 *   -f_min         f(0.1928, 0.1908, 0.1231, 0.1358) ≈ 0.0003075
	 *   -brief
	 * @param x 自变量
	 * @param d 维数
	 * @return 函数值
	*/
	static double Kowalik(double* x, int d = 4) {
		double a[11] = { 0.1957, 0.1947, 0.1735, 0.1600, 0.0844,
			0.0627, 0.0456, 0.0342, 0.0323, 0.0235, 0.0246 };
		double b[11] = { 4.0, 2.0, 1.0, 0.5, 0.25,
			1.0 / 6.0, 1.0 / 8.0, 0.1, 1.0 / 12.0, 1.0 / 14.0, 1.0 / 16.0};
		double s = 0.0;
		for (int i = 0; i < 11; i++) {
			s += pow(a[i] - (x[0] * (b[i] * b[i] + b[i] * x[1])) / (b[i] * b[i] + b[i] * x[2] + x[3]), 2);
		}
		Benchmark::T++;
		return s;
	}

	/**
	 * @brief No.16
	 *   -name          Six-Hump Camel-Back Function
	 *   -formula       https://wenku.baidu.com/view/97a0880102020740be1e9b86.html No.16
	 *   -Search Space  [-5, 5]^2
	 *   -f_min         f(0.08983, -0.7126) = f(-0.08983, 0.7126) = -1.0316285
	 *   -brief
	 * @param x 自变量
	 * @param d 维数
	 * @return 函数值
	*/
	static double Camel_Back(double* x, int d = 2) {
		double x12 = x[0] * x[0];
		double x14 = x12 * x12;
		double x16 = x14 * x12;
		double x22 = x[1] * x[1];
		double x24 = x22 * x22;
		Benchmark::T++;
		return 4.0 * x12 - 2.1 * x14 + 1.0 / 3.0 * x16 + x[0] * x[1] - 4.0 * x22 + 4.0 * x24;
	}

	/**
	 * @brief No.17
	 *   -name          Branin Function
	 *   -formula       https://wenku.baidu.com/view/97a0880102020740be1e9b86.html No.17
	 *   -Search Space  x1-[-5, 10]   x2-[5, 15]
	 *   -f_min         f(-3.142, 12.275) = f(3.142, 2.275) = f(9.425, 2.425) = 0.398
	 *   -brief
	 * @param x 自变量
	 * @param d 维数
	 * @return 函数值
	*/
	static double Branin(double* x, int d = 2) {
		double x1 = x[0], x2 = x[1];
		double p1 = pow(x2 - 5.1 / (4.0 * PI * PI) * x1 * x1 + 5.0 / PI * x1 - 6.0, 2);
		double p2 = 10.0 * (1.0 - 1.0 / 8.0 / PI) * cos(x1);
		Benchmark::T++;
		return p1 + p2 + 10;
	}

	/**
	 * @brief No.18
	 *   -name          Goldstein-Price Function
	 *   -formula       https://wenku.baidu.com/view/97a0880102020740be1e9b86.html No.18
	 *   -Search Space  [-2, 2]
	 *   -f_min         f(0, -1) = 3
	 *   -brief
	 * @param x 自变量
	 * @param d 维数
	 * @return 函数值
	*/
	static double Goldstein_Price(double* x, int d = 2) {
		double x1 = x[0], x2 = x[1];
		double x12 = x1 * x1, x22 = x2 * x2;
		double p1 = 1.0 + pow(x1 + x2 + 1.0, 2) * 
			(19.0 - 14.0 * x1 + 3.0 * x12 - 14.0 * x2 + 6.0 * x1 * x2 + 3.0 * x22);
		double p2 = 30.0 + pow(2.0 * x1 - 3.0 * x2, 2) *
			(18.0 - 32.0 * x1 + 12.0 * x12 + 48.0 * x2 - 36.0 * x1 * x2 + 27.0 * x22);
		Benchmark::T++;
		return p1 * p2;
	}

	/**
	 * @brief No.19
	 *   -name          Neumaire 3 Function
	 *   -formula       https://wenku.baidu.com/view/97a0880102020740be1e9b86.html No.21
	 *   -Search Space  [-D^2, D^2]^D
	 *   -f_min         f() = 0
	 *   -brief
	 * @param x 自变量
	 * @param d 维数
	 * @return 函数值
	*/
	static double Neumaire(double* x, int d) {
		double s1 = 0.0, s2 = 0.0;
		for (int i = 0; i < d; i++) {
			s1 += pow(x[i] - 1.0, 2);
		}
		for (int i = 1; i < d; i++) {
			s2 += x[i] * x[i - 1];
		}
		Benchmark::T++;
		return s1 + s2 + d * (d + 1.0) * (d - 1.0) / 6.0;
	}

	/**
	 * @brief No.20
	 *   -name          Salomon Function
	 *   -formula       https://wenku.baidu.com/view/97a0880102020740be1e9b86.html No.22
	 *   -Search Space  [-100, 100]^D
	 *   -f_min         f() = 0
	 *   -brief
	 * @param x 自变量
	 * @param d 维数
	 * @return 函数值
	*/
	static double Salomon(double* x, int d) {
		double s = 0.0;
		for (int i = 0; i < d; i++) {
			s += x[i] * x[i];
		}
		Benchmark::T++;
		return 1.0 - cos(2.0 * PI * sqrt(s)) + 0.1 * sqrt(s);
	}

	/**
	 * @brief No.21
	 *   -name          Alpine Function
	 *   -formula       https://wenku.baidu.com/view/97a0880102020740be1e9b86.html No.23
	 *   -Search Space  [-10, 10]^D
	 *   -f_min         f() = 0
	 *   -brief
	 * @param x 自变量
	 * @param d 维数
	 * @return 函数值
	*/
	static double Alpine(double* x, int d) {
		double s = 0.0;
		for (int i = 0; i < d; i++) {
			s += abs(x[i] * sin(x[i]) + 0.1 * x[i]);
		}
		Benchmark::T++;
		return s;
	}

private:
	/**
	 * @brief  Generalized Penalized Functions 中的 u
	 *
	 *             | k(x-a)^m,        x>a
	 * u(x,a,k,m)= | 0,               -a<=x<=a
	 *             | k(-x-a)^m,       x<-a
	 *
	*/
	static double getu(double x, double a, double k, double m) {
		if (x > a) {
			return k * pow(x - a, m);
		}
		else if (x < -a) {
			return k * pow(-x - a, m);
		}
		else {
			return 0;
		}
	}

	/**
	 * @brief  Generalized Penalized Functions 中的 y
	 *
	 * y = 1 + 1 / 4 * (x + 1);
	 *
	*/
	static double gety(double x) {
		return 1 + 1 / 4 * (x + 1);
	}
};
