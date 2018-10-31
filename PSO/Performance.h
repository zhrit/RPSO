/***************************************************************
*  本文件包含PSO算法性能测试相关函数                                *
*  @author   ZhRiT                                             *
*  @email    buaazhangrit@163.com                              *
*  @version  1.0.0                                             *
*  @date     2018-09-20                                        *
***************************************************************/
#pragma once

#include <iostream>
#include <fstream>
#include <streambuf>
#include <string>
#include <vector>
#include <ctime>

/**
 * PSO算法性能相关参数
 */
struct PerfInfo {
	int PopulationSize; // 粒子个数
	int Iterations;     // （平均）迭代次数
	int Iterations_S;   // （平均）成功样本的迭代次数
	double Time;        // （平均）运行时间
	double Time_S;      // （平均）成功样本的运行时间
	double Time_O;      // （平均）成功样本的目标函数执行次数
	double SuccessRate; // 成功率
	int Repeat;         // 执行次数
	int dem;            // 维数
};

/**
 * @brief 性能测试类
 */
class Performance {
public:
	Performance();
	~Performance() {}

	/**
	 * @breif 给定维度，给定粒子总数，测试性能
	 * @ param serial 目标函数序号
	 * @ param d      维数
	 * @ param min    自变量下界
	 * @ param min    自变量上界
	 * @ param n      粒子个数
	 * @ param repeat 重复次数
	 * @ param tmax   最大迭代次数
	 */
	PerfInfo SingleTest(int serial, int d, double *min, double *max, int n, int repeat = 30, int tmax = 3000);

	/**
	 * @breif 给定维度，粒子总数对性能的影响
	 * @ param serial   目标函数序号
	 * @ param d        维数
	 * @ param min      自变量下界
	 * @ param min      自变量上界
	 * @ param repeat   重复次数
	 * @ param tmax     最大迭代次数
	 * @ param fileName 保存数据的文件名
	 * @ param n_l      粒子总数范围下界
	 * @ param n_u      粒子总数范围上界
	 */
	void EffectOfPopulation(int serial, int d, double *min, double *max, int repeat = 30, int tmax = 3000, string fileName = "", int n_l = 3, int n_u = 30);

	/**
	 * @breif 不同维度维度，粒子总数对性能的影响
	 * @ param serial   目标函数序号
	 * @ param min      自变量下界
	 * @ param min      自变量上界
	 * @ param repeat   重复次数
	 * @ param tmax     最大迭代次数
	 * @ param fileName 保存数据的文件名
	 */
	void EffectOfPopulation_D(int serial, double *min, double *max, int repeat = 30, int tmax = 3000, string fileName = "");

	/**
	 * @brief 把结果写入excel文件
	 * @param infos 运行结果
	 */
	void WriteToCSV(const vector<PerfInfo> &infos, const string fileName);

private:
	// 测试函数池
	double (*FuncPool[21])(double*, int) = { Benchmark::Sphere, Benchmark::Schwefel1_2, 
		Benchmark::Schwefel2_22, Benchmark::Schwefel2_21, Benchmark::Generalized, Benchmark::Step, Benchmark::Quartic,
		Benchmark::Generalized_Schwefel2_26, Benchmark::Generalized_Rastrigin, Benchmark::Ackley,
		Benchmark::Generalized_Griewank, Benchmark::Generalized_Penalized_1, Benchmark::Generalized_Penalized_2,
		Benchmark::Shekel_Foxholes, Benchmark::Kowalik, Benchmark::Camel_Back, Benchmark::Branin,
		Benchmark::Goldstein_Price, Benchmark::Neumaire, Benchmark::Salomon, Benchmark::Alpine };
	// 测试函数理论最优值
	double FuncOptimals[21] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -418.9828872724338, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.998004, 0.0003075, -1.0316285, 0.397887364, 3.0,
		0.0, 0.0, 0.0};
	// 各个测试函数对应的精度
	double FuncPrec[21] = { 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6,
		1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6 };
	// 测试函数搜索空间大小
	double FuncSpaceSize[21] = { 100, 100, 10, 100, 30, 100, 1.28, 500, 5.12,
		32, 600, 50, 50, 65.536, 5, 5, 15, 2, 1, 100, 10
	};

	bool HasSetRandomSeed{ false }; // 是否设置过随机因子

	/**
	 * 设置随机树种子
	 */
	void SetRandomSeed();
};