/***************************************************************
*  本文件包含粒子群算法类的定义，是本程序核心类                        *
*  @author   ZhRiT                                             *
*  @email    buaazhangrit@163.com                              *
*  @version  1.0.0                                             *
*  @date     2018-09-21                                        *
***************************************************************/

#include "pch.h"
#include "PSO.cuh"
#include <windows.h>  
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <ctime>
#include <string>
#include <vector>
using namespace std;

/*----------------------------------------------------------粒子群优化算法类定义--------------------------------------------------------------*/

/**
 * @brief 算法初始化
 *   传入必要的参数
 * @param obj  目标函数
 * @param d    维数
 * @param min  下界
 * @param max  上界
 * @param n    粒子个数
 * @param t    最大迭代次数
 * @param opt  理论最优值，测试模式下用到
 * @param prec 精度，测试模式下用到
 */
void PSO::Initialize(double (*obj)(double*, int), int d, double *min, double *max, int n, int t, double opt, double prec) {
	m_objFun = obj;
	m_d = d;
	m_number = n;
	m_tMax = t;
	m_min = min;
	m_max = max;
	m_opt_theo = opt;
	m_precision = prec;

	m_result_value_ite = new double[m_tMax];
	m_result_pos = new double[m_d];
}

/**
 * @brief 启动算法，完成算法主要逻辑
 */
void PSO::Run() {
	/********** 预处理 **********/
	// 目标函数执行次数设置为0
	Benchmark::T = 0;
	m_t_obj = 0;
	/********** 环境监测 **********/
	if (m_d == 0) {
		cout << "请通过Initialize()方法或SetD()方法给定优化问题的维度！" << endl;
		m_status = INPUTERROR;
	}
	else {}
	if (m_objFun == nullptr) {
		cout << "请通过Initialize()方法或SetObjFun()方法给定优化问题的目标函数！" << endl;
		m_status = INPUTERROR;
	}
	if (m_min == nullptr || m_max == nullptr) {
		cout << "请通过Initialize()方法或SetMin()和SetMax()方法给定优化问题的目标函数！" << endl;
		m_status = INPUTERROR;
	}

	/********** 算法状态输出 **********/
	string topologyStr[3] = { "全局拓扑", "环形拓扑", "随机拓扑" };
	cout << "=====算法启动！=====" << endl;
	cout << "=====" << (m_controller[string("model")] ? "求解模式" : "测试模式") << "==";
	cout << topologyStr[m_controller[string("topology")]] << "==";
	cout << (m_controller[string("minmax")] ? "最大值问题" : "最小值问题") << "==";
	cout << (m_controller[string("parallel")] ? "GPU" : "CPU") << "=====" << endl;
	cout << "最大迭代次数：" << m_tMax << "，  粒子个数：" << m_number << "，  问题维度：" << m_d << "。" << endl;

	double beginTime = GetTickCount();
	if (m_controller[string("parallel")] == 0) {
		Run_CPU();
	}
	else {
		Run_GPU();
	}

	double endTime = GetTickCount();
	m_time_cost = endTime - beginTime;

	cout << "=====结束！=====" << endl;
	Output();
}

/**
 * @brief 在CPU上运行
 */
void PSO::Run_CPU() {
	// 常数初始化
	const double c1 = 2.05, c2 = 2.05;
	const double phi = c1 + c2;
	const double chi = 2.0 / abs(2.0 - phi - sqrt(phi * phi - 4.0 * phi));

	// 初始化粒子
	//double* r11 = new double[m_d];
	//double* r22 = new double[m_d];
	//cout << "=====初始化粒子！=====" << endl;
	m_Particles = new Particle[m_number];
	for (int i = 0; i < m_number; i++) {
		(m_Particles + i)->Initialize(m_d, m_min, m_max, m_objFun);
		// cout << *(m_Particles + i) << endl;
	}
	// 主循环
	for (int t = 0; t <= m_tMax; t++) {
		// 更新个体最优位置和个体最优值
		for (int i = 0; i < m_number; i++) {
			(m_Particles + i)->UpdateBest(m_controller["minmax"]);
		}
		// 更新全局最优位置和全局最优值
		_UpdateGlobalBest(t);
		// 判断是否达到精度和退出循环的条件
		if (m_controller[string("model")] == 0) {
			if (abs(m_result_value - m_opt_theo) <= m_precision) {
				m_t_act = t;
				m_status = SUCCESS;
				break;
			}
			else if (t == m_tMax) {
				m_t_act = t;
				m_status = FAIL;
				break;
			}
		}
		else {
			if (t == m_tMax) {
				m_t_act = t;
				m_status = OK;
				break;
			}
		}
		// 重采样
		if (m_controller[string("useResample")] == 1) {
			_Resample(t);
		}
		// 速度更新
		//double r1 = rand() / (double)(RAND_MAX);
		//double r2 = rand() / (double)(RAND_MAX);
		//for (int i = 1; i < m_d; i++) {
		//	r11[i] = rand() / (double)(RAND_MAX);
		//	r22[i] = rand() / (double)(RAND_MAX);
		//}
		if (m_controller[string("topology")] == 0) {
			for (int i = 0; i < m_number; i++) {
				//double r1 = rand() / (double)(RAND_MAX);
				//double r2 = rand() / (double)(RAND_MAX);
				(m_Particles + i)->Move(chi, c1, c2, m_result_pos);
			}
		}
		else {
			_UpdateLocalBest();
			for (int i = 0; i < m_number; i++) {
				//double r1 = rand() / (double)(RAND_MAX);
				//double r2 = rand() / (double)(RAND_MAX);
				(m_Particles + i)->Move(chi, c1, c2, nullptr);
			}
		}
	}

	m_t_obj = Benchmark::T;
	// 回收内存
	delete[] m_Particles;
	m_Particles = nullptr;
	//delete[] r11;
	//r11 = nullptr;
	//delete[] r22;
	//r22 = nullptr;
}

/**
 * @brief 算法结果输出到控制台
 */
void PSO::Output() const {
	cout << "=====结果！=====" << endl;
	cout << "最优点：";
	for (int i = 0; i < m_d; i++) {
		cout << m_result_pos[i] << ", ";
	}
	cout << endl << "最优值：" << m_result_value << endl;
	cout << "实际迭代次数：" << m_t_act << endl;
	cout << "实际消耗时间：" << m_time_cost << endl;
	cout << "目标函数运行次数：" << Benchmark::T << endl;
	cout << "结果状态值：" << m_status << endl;
}

/**
 * @brief 算法结果绘图
 */
void PSO::Draw() const {
	//TODO
}

/**
 * @brief 逐个设置算法控制器属性
 * @param attr   控制器名称
 * @param value  控制器的值
 */
void PSO::SetOption(const char* attr, int value) {
	if (m_controller.count(attr) == 0) {
		cout << "没有此属性！" << endl;
		return;
	}
	m_controller[attr] = value;
}

/*----- 属性存取器 -----*/
void PSO::SetTMax(int t) {
	m_tMax = t;
}
int PSO::GetTMax() const {
	return m_tMax;
}

void PSO::SetN(int n) {
	m_number = n;
}
int PSO::GetN(int) const {
	return m_number;
}

void PSO::SetD(int d) {
	m_d = d;
}
int PSO::GetD() const {
	return m_d;
}

void PSO::SetObjFun(double(*obj)(double*, int)) {
	m_objFun = obj;
}

void PSO::SetMin(double* min) {
	m_min = min;
}
double *PSO::GetMin() const {
	return m_min;
}

void PSO::SetMax(double* max) {
	m_max = max;
}
double *PSO::GetMax() const {
	return m_max;
}

void PSO::SetOptTheo(double opt) {
	m_opt_theo = opt;
}
double PSO::GetOptTheo() const {
	return m_opt_theo;
}

void PSO::SetPrec(double prec) {
	m_precision = prec;
}
double PSO::GetPrec() const {
	return m_precision;
}

double PSO::GetTimeCost() const {
	return m_time_cost;
}

int PSO::GetTAct() const {
	return m_t_act;
}

int PSO::GetTObj() const {
	return m_t_obj;
}

Status PSO::GetStatus() const {
	return m_status;
}

/*----- 内部方法 -----*/

/**
 * @brief 更新全局最优
 * @param t_current  当前迭代次数
 */
void PSO::_UpdateGlobalBest(int t_current) {
	for (int i = 0; i < m_number; i++) {
		if (i == 0 && t_current == 0) {
			m_result_value = (m_Particles + i)->m_Value_best;
			for (int j = 0; j < m_d; j++) {
				m_result_pos[j] = (m_Particles + i)->m_Pos_best[j];
			}
		}
		else {
			if (m_controller["minmax"] == 0) {
				if ((m_Particles + i)->m_Value_best < m_result_value) {
					m_result_value = (m_Particles + i)->m_Value_best;
					for (int j = 0; j < m_d; j++) {
						m_result_pos[j] = (m_Particles + i)->m_Pos_best[j];
					}
				}
			}
			else {
				if ((m_Particles + i)->m_Value_best > m_result_value) {
					m_result_value = (m_Particles + i)->m_Value_best;
					for (int j = 0; j < m_d; j++) {
						m_result_pos[j] = (m_Particles + i)->m_Pos_best[j];
					}
				}
			}
		}
	}
	m_result_value_ite[t_current] = m_result_value;
}

/**
 * @brief 更新局部最优
 */
void PSO::_UpdateLocalBest() {
	if (m_controller[string("topology")] == 1) {
		// 环形拓扑
		if (m_controller["minmax"] == 0) {
			for (int i = 0; i < m_number; i++) {
				int lp = i - 1, rp;
				if (lp < 0) { lp += m_number; }
				rp = (i + 1) % m_number;
				if ((m_Particles + lp)->GetSelfBestValue() < (m_Particles + i)->GetLocalBestValue()) {
					(m_Particles + i)->SetLocalBest((m_Particles + lp)->GetSelfBestPos(), (m_Particles + lp)->GetSelfBestValue());
				}
				if ((m_Particles + rp)->GetSelfBestValue() < (m_Particles + i)->GetLocalBestValue()) {
					(m_Particles + i)->SetLocalBest((m_Particles + rp)->GetSelfBestPos(), (m_Particles + rp)->GetSelfBestValue());
				}
			}
		}
		else {
			for (int i = 0; i < m_number; i++) {
				int lp = i - 1, rp;
				if (lp < 0) { lp += m_number; }
				rp = (i + 1) % m_number;
				if ((m_Particles + lp)->GetSelfBestValue() > (m_Particles + i)->GetLocalBestValue()) {
					(m_Particles + i)->SetLocalBest((m_Particles + lp)->GetSelfBestPos(), (m_Particles + lp)->GetSelfBestValue());
				}
				if ((m_Particles + rp)->GetSelfBestValue() > (m_Particles + i)->GetLocalBestValue()) {
					(m_Particles + i)->SetLocalBest((m_Particles + rp)->GetSelfBestPos(), (m_Particles + rp)->GetSelfBestValue());
				}
			}
		}
	}
	else if (m_controller[string("topology")] == 2) {
		// 随机拓扑
		if (m_controller["minmax"] == 0) {
		}
		else {

		}
	}
	else {

	}
}

/**
 * @brief 重采样
 */
void PSO::_Resample(int t) {
	switch (m_controller[string("resampleMethod")]) {
	case 0:
		_StanResample(t);
		break;
	case 1:
		_MultResample(t);
		break;
	case 2:
		_StraResample(t);
		break;
	case 3:
		_SystResample(t);
		break;
	case 4:
		_ResiResample(t);
		break;
	case 5:
		_ResiSystResample(t);
		break;
	default:
		break;
	}
}

/**
 * @brief 标准重采样
 * @param 当前迭代次数
 */

void PSO::_StanResample(int t) {
	// 分配权重
	double* weights = new double[m_number];
	_GetWeight(weights);

	// 重采样条件判断
	double Neff = 0.0;
	for (int i = 1; i < m_number; i++) {
		Neff += weights[i] * weights[i];
	}
	Neff = 1 / Neff;

	if (Neff > 6.6) {
		delete[] weights;
		weights = nullptr;
		return;
	}

	// 权重的均值
	double w_mean = 0.0;
	for (int i = 0; i < m_number; i++) {
		w_mean += weights[i];
	}
	w_mean = w_mean / double(m_number);

	// 权重的方差
	double w_var = 0.0;
	for (int i = 0; i < m_number; i++) {
		w_var += pow(weights[i] - w_mean, 2);
	}
	w_var /= double(m_number);

	// 阈值
	double w_t = w_mean - 2 * w_var;
	// 权重小的重采样，随机生成新的粒子
	for (int i = 0; i < m_number; i++) {
		if (weights[i] < w_t) {
			//cout << i << ", ";
			(m_Particles + i)->Random();
			(m_Particles + i)->InitBest();
		}
	}
	//cout << endl;

	// 释放内存
	delete[] weights;
	weights = nullptr;
}

/**
 * @brief 多项式重采样
 */
void PSO::_MultResample(int t) {
	// 分配权重
	double* weights = new double[m_number];
	_GetWeight(weights);

	// 重采样条件判断
	double Neff = 0.0;
	for (int i = 1; i < m_number; i++) {
		Neff += weights[i] * weights[i];
	}
	Neff = 1 / Neff;

	if (Neff > 6.6) {
		delete[] weights;
		weights = nullptr;
		return;
	}

	// 权重一次求和
	for (int i = 1; i < m_number; i++) {
		weights[i] += weights[i - 1];
	}

	// 取随机数
	int *selected = new int[m_number];
	int *selected_sort = new int[m_number];
	for (int i = 0; i < m_number; i++) {
		double u = rand() / double(RAND_MAX);
		int m = 0;
		while (weights[m] < u) {
			m++;
		}
		selected[i] = m;
	}

	// 整理selcted，获取重采样结果的数组表示
	_SelectSort(selected, selected_sort);

	// 根据重采样结果数组更新粒子
	_UpdateParticles(selected_sort, t);


	// 释放内存
	delete[] selected;
	selected = nullptr;
	delete[] selected_sort;
	selected_sort = nullptr;
	delete[] weights;
	weights = nullptr;
}

/**
 * @brief 分层重采样
 */
void PSO::_StraResample(int t) {
	// 分配权重
	double* weights = new double[m_number];
	_GetWeight(weights);

	// 重采样条件判断
	double Neff = 0.0;
	for (int i = 1; i < m_number; i++) {
		Neff += weights[i] * weights[i];
	}
	Neff = 1 / Neff;

	if (Neff > 6.6) {
		delete[] weights;
		weights = nullptr;
		return;
	}

	// 权重一次求和
	for (int i = 1; i < m_number; i++) {
		weights[i] += weights[i - 1];
	}

	// 取随机数
	int m = 0;

	int *selected = new int[m_number];
	int *selected_sort = new int[m_number];
	for (int i = 0; i < m_number; i++) {
		double u0 = rand() / double(RAND_MAX) / double(m_number);
		double u = u0 + double(i) / double(m_number);
		while (weights[m] < u) {
			m++;
		}
		selected[i] = m;
	}

	// 整理selcted，获取重采样结果的数组表示
	_SelectSort(selected, selected_sort);

	// 根据重采样结果数组更新粒子
	_UpdateParticles(selected_sort, t);


	// 释放内存
	delete[] selected;
	selected = nullptr;
	delete[] selected_sort;
	selected_sort = nullptr;
	delete[] weights;
	weights = nullptr;
}

/**
 * @brief 系统重采样
 */
void PSO::_SystResample(int t) {
	// 分配权重
	double* weights = new double[m_number];
	_GetWeight(weights);

	// 重采样条件判断
	double Neff = 0.0;
	for (int i = 1; i < m_number; i++) {
		Neff += weights[i] * weights[i];
	}
	Neff = 1 / Neff;

	if (Neff > 6.6) {
		delete[] weights;
		weights = nullptr;
		return;
	}

	// 权重一次求和
	for (int i = 1; i < m_number; i++) {
		weights[i] += weights[i - 1];
	}

	// 取随机数
	int m = 0;

	int *selected = new int[m_number];
	int *selected_sort = new int[m_number];
	double u0 = rand() / double(RAND_MAX) / double(m_number);
	for (int i = 0; i < m_number; i++) {
		double u = u0 + double(i) / double(m_number);
		while (weights[m] < u) {
			m++;
		}
		selected[i] = m;
	}

	// 整理selcted，获取重采样结果的数组表示
	_SelectSort(selected, selected_sort);

	// 根据重采样结果数组更新粒子
	_UpdateParticles(selected_sort, t);


	// 释放内存
	delete[] selected;
	selected = nullptr;
	delete[] selected_sort;
	selected_sort = nullptr;
	delete[] weights;
	weights = nullptr;
}

/**
 * @brief 残差重采样
 */
void PSO::_ResiResample(int t) {
	// 分配权重
	double* weights = new double[m_number];
	_GetWeight(weights);

	// 重采样条件判断
	double Neff = 0.0;
	for (int i = 1; i < m_number; i++) {
		Neff += weights[i] * weights[i];
	}
	Neff = 1 / Neff;

	if (Neff > 6.6) {
		delete[] weights;
		weights = nullptr;
		return;
	}

	int *selected = new int[m_number]; // 重采样数组

	int selectedIndex = 0; // selected数组填充位置标记

	// 权重大于1/N的复制，复制次数为floor（N * w）
	for (int i = 0; i < m_number; i++) {
		double num4copy = floor(m_number * weights[i]);// 计算复制次数
		weights[i] = weights[i] - num4copy / double(m_number); // 新权重
		// 填充selected数组
		for (int j = 0; j < num4copy; j++) {
			selected[selectedIndex] = i;
			selectedIndex++;
		}
	}

	// 如果复制的粒子总数小于种群粒子数，则用多项式重采样方法取剩下的粒子
	if (selectedIndex < m_number) {
		// 新权重归一化
		for (int i = 0; i < m_number; i++) {
			weights[i] = weights[i] * double(m_number) / double(m_number - selectedIndex);
		}
		// 新权重一次求和
		for (int i = 1; i < m_number; i++) {
			weights[i] += weights[i - 1];
		}
		// 多项式重采样
		while (selectedIndex < m_number) {
			double u = rand() / double(RAND_MAX);
			int m = 0;
			while (weights[m] < u) {
				m++;
			}
			selected[selectedIndex] = m;
			selectedIndex++;
		}
	}

	int *selected_sort = new int[m_number];

	// 整理selcted，获取重采样结果的数组表示
	_SelectSort(selected, selected_sort);

	// 根据重采样结果数组更新粒子
	_UpdateParticles(selected_sort, t);


	// 释放内存
	delete[] selected;
	selected = nullptr;
	delete[] selected_sort;
	selected_sort = nullptr;
	delete[] weights;
	weights = nullptr;
}

/**
 * @brief 残差系统重采样
 * @param 当前迭代次数
 */
void PSO::_ResiSystResample(int t) {
	// 分配权重
	double* weights = new double[m_number];
	_GetWeight(weights);

	// 重采样条件判断
	double Neff = 0.0;
	for (int i = 1; i < m_number; i++) {
		Neff += weights[i] * weights[i];
	}
	Neff = 1 / Neff;

	if (Neff > 6.6) {
		delete[] weights;
		weights = nullptr;
		return;
	}

	int *selected = new int[m_number]; // 重采样数组

	int selectedIndex = 0; // selected数组填充位置标记

	double u0 = rand() / double(RAND_MAX) / double(m_number);

	// 权重大于1/N的复制，复制次数为floor（N * w）
	for (int i = 0; i < m_number; i++) {
		double num4copy = floor(m_number * (weights[i] - u0)) + 1;// 计算复制次数
		u0 = u0 + num4copy / double(m_number) - weights[i];
		// 填充selected数组
		for (int j = 0; j < num4copy; j++) {
			selected[selectedIndex] = i;
			selectedIndex++;
		}
	}
	cout << selectedIndex << endl;

	int *selected_sort = new int[m_number];

	// 整理selcted，获取重采样结果的数组表示
	_SelectSort(selected, selected_sort);

	// 根据重采样结果数组更新粒子
	_UpdateParticles(selected_sort, t);


	// 释放内存
	delete[] selected;
	selected = nullptr;
	delete[] selected_sort;
	selected_sort = nullptr;
	delete[] weights;
	weights = nullptr;
}

/**
 * @brief 获取每个粒子的权重
 * @param w 指向权重数组的指针
 */
void PSO::_GetWeight(double* w) {
	// 和最优值的差距
	double sum = 0.0;
	for (int i = 0; i < m_number; i++) {
		w[i] = abs(m_Particles[i].m_Value - m_result_value);
		sum += w[i];
	}
	// 求均值方差
	double mean = sum / double(m_number);
	double var = 0.0;
	for (int i = 0; i < m_number; i++) {
		var += pow(w[i] - mean, 2);
	}
	var /= double(m_number);
	// 求权重
	for (int i = 0; i < m_number; i++) {
		w[i] = 1.0 / sqrt(2 * PI * var) * exp(-w[i] * w[i] / (2 * var));
	}
	// 权重归一化
	sum = 0.0;
	for (int i = 0; i < m_number; i++) {
		sum += w[i];
	}
	for (int i = 0; i < m_number; i++) {
		w[i] /= sum;
	}
}

/**
 * @brief 随机重采样
 * @param selected 代表采样关系的指针
 */
void PSO::_SelectSort(int* selected, int* selected_sort) {
	// seleted是选出来的粒子，seleted_sort是对选出粒子的排序，为的是尽量保持原有的顺序
	for (int i = 0; i < m_number; i++) {
		selected_sort[i] = -1;
	}

	for (int i = 0; i < m_number; i++) {
		if (selected_sort[selected[i]] == -1) {
			selected_sort[selected[i]] = selected[i];
			selected[i] = -1;
		}
	}
	for (int i = 0; i < m_number; i++) {
		if (selected[i] != -1) {
			for (int j = 0; j < m_number; j++) {
				if (selected_sort[j] == -1) {
					selected_sort[j] = selected[i];
					break;
				}
			}
		}
	}
}

/**
 * @brief 根据重采样结果数组更新粒子
 * @param s 重采样结果数组
 */
void PSO::_UpdateParticles(const int* s, int t) {
	for (int i = 0; i < m_number; i++) {
		if (s[i] != i) {
			cout << i << "  " << s[i] << endl;
			_Copy4Resample(m_Particles + i, m_Particles + s[i], t);
		}
	}
}

/**
 * @brief  重采样时，把另一个粒子的数据拷贝过来
 * @param  拷贝粒子的指针
 * @param  被拷贝的粒子的指针
 * @param  当前迭代次数
 */
void PSO::_Copy4Resample(Particle* p1, const Particle* p2, int t) {
	// 当前位置-复制
	for (int i = 0; i < m_d; i++) {
		p1->m_Pos[i] = p2->m_Pos[i];
		// p1->m_Vec[i] = p2->m_Vec[i];
	}
	// 当前位置对应的目标函数值-复制
	p1->m_Value = p2->m_Value;

	// 当前速度-合成
	for (int i = 0; i < m_d; i++) {
		double x_r = rand() / double(RAND_MAX) * (m_max[i] - m_min[i]) + m_min[i];
		double v_r = rand() / double(RAND_MAX) * (m_max[i] - m_min[i]) + m_min[i] - x_r;
		/*p1->m_Vec[i] = double(m_tMax + t) / (2.0 * double(m_tMax)) * v_r +
			double(m_tMax - t) / (2.0 * double(m_tMax)) * p2->m_Vec[i];*/
		p1->m_Vec[i] = double(m_tMax + t) / (2.0 * double(m_tMax)) * p2->m_Vec[i] +
			double(m_tMax - t) / (2.0 * double(m_tMax)) * v_r;
	}

	// 个体最优位置、个体最优值、局部最优位置、局部最优值-取最好
	if (m_controller["minmax"] == 0) {
		if (p2->m_Value_best < p1->m_Value_best) {
			p1->m_Value_best = p2->m_Value_best;
			p1->m_Value_best_local = p2->m_Value_best_local;
			for (int i = 0; i < m_d; i++) {
				p1->m_Pos_best[i] = p2->m_Pos_best[i];
				p1->m_Pos_best_local[i] = p2->m_Pos_best_local[i];
			}
		}
	}
	else {
		if (p2->m_Value_best > p1->m_Value_best) {
			p1->m_Value_best = p2->m_Value_best;
			p1->m_Value_best_local = p2->m_Value_best_local;
			for (int i = 0; i < m_d; i++) {
				p1->m_Pos_best[i] = p2->m_Pos_best[i];
				p1->m_Pos_best_local[i] = p2->m_Pos_best_local[i];
			}
		}
	}
}
