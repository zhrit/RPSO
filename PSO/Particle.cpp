/***************************************************************
*  本文件包含粒子类的定义                                          *
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

/*----------------------------------------------------------粒子类定义--------------------------------------------------------------*/
Particle::Particle() {
	// 重新设置随机种子，在random()方法中用到了随机数
	// srand((unsigned int)time(NULL));
}

Particle::~Particle() {
	// 释放空间，指针置空
	if (!isNull) {
		delete[] m_Pos;
		delete[] m_Vec;
		delete[] m_Pos_best;
		delete[] m_Pos_best_local;
	}
	m_Pos = nullptr;
	m_Vec = nullptr;
	m_Pos_best = nullptr;
	m_Pos_best_local = nullptr;
	m_min = nullptr;
	m_max = nullptr;
}

/**
 * @brief 初始化粒子
 *   在粒子群算法对每个粒子调用，初始化粒子信息；
 *   包括随机产生位置、速度、初始化个体最优和局部最优等操作
 * @param d      维数
 * @param min    位置下界
 * @param max    位置上界
 * @param objFun 目标函数
 * @return
 */
void Particle::Initialize(int d, double* min, double* max, double(*objFun)(double*, int)) {
	isNull = false;
	m_Size = d;
	m_Pos = new double[d];
	m_Vec = new double[d];
	m_Pos_best = new double[d];
	m_Pos_best_local = new double[d];
	m_objFun = objFun;
	m_min = min;
	m_max = max;
	Random();
	InitBest();
}

/**
 * @brief 随机产生粒子的位置和速度
 */
void Particle::Random() {
	for (int i = 0; i < m_Size; i++) {
		m_Pos[i] = rand() / double(RAND_MAX) * (m_max[i] - m_min[i]) + m_min[i];
		m_Vec[i] = rand() / double(RAND_MAX) * (m_max[i] - m_min[i]) + m_min[i] - m_Pos[i];
	}
	m_Value = CalValue();
}

/**
 * @brief 给个体最优和局部最优赋初值
 *   个体最优在速度和位置更新中用到
 *   局部最优在环形拓扑和随机拓扑中用到
 */
void Particle::InitBest() {
	for (int i = 0; i < m_Size; i++) {
		m_Pos_best[i] = m_Pos[i];
		m_Pos_best_local[i] = m_Pos[i];
	}
	m_Value_best = m_Value;
	m_Value_best_local = m_Value;
}

/**
 * @brief 计算当前位置坐标对应的目标函数值
 */
double Particle::CalValue() const {
	return m_objFun(m_Pos, m_Size);
}

/**
 * @brief 更新个体最优和局部最优
 * @param index min_max优化标志
 *   -0 min优化
 *   -1 max优化
 */
void Particle::UpdateBest(int index) {
	double newValue = m_Value;
	if (index == 0) {
		if (newValue < m_Value_best) {
			m_Value_best = newValue;
			m_Value_best_local = newValue;
			for (int i = 0; i < m_Size; i++) {
				m_Pos_best[i] = m_Pos[i];
				m_Pos_best_local[i] = m_Pos[i];
			}
		}
	}
	else {
		if (newValue > m_Value_best) {
			m_Value_best = newValue;
			m_Value_best_local = newValue;
			for (int i = 0; i < m_Size; i++) {
				m_Pos_best[i] = m_Pos[i];
				m_Pos_best_local[i] = m_Pos[i];
			}
		}
	}
}

/**
 * @brief 粒子向前移动一步
 * @param chi 压缩因子
 * @param c1  加速度系数（个体）
 * @param c2  加速度系数（群体）
 * @param r1  随机因子（个体）
 * @param r2  随机因子（群体）
 * @param gb  群体对个体产生影响的位置
 *   -nullptr 随机拓扑或环形拓扑时传入nullptr，此时自动使用局部最优位置
 *   -gb      全局拓扑时，讲全局最优传入
 */
 // 每个粒子的每一维用相同的数字r1、r2
void Particle::Move(double chi, double c1, double c2, double r1, double r2, const double *gb) {
	if (gb == nullptr) gb = m_Pos_best_local;
	for (int i = 0; i < m_Size; i++) {
		m_Vec[i] = chi * (m_Vec[i] + c1 * r1 * (m_Pos_best[i] - m_Pos[i]) + c2 * r2 * (gb[i] - m_Pos[i]));
		m_Pos[i] += m_Vec[i];
		// 超出范围的处理
		if (m_Pos[i] < m_min[i]) {
			m_Pos[i] = m_min[i];
			m_Vec[i] = -m_Vec[i] * 0.5;
		}
		if (m_Pos[i] > m_max[i]) {
			m_Pos[i] = m_max[i];
			m_Vec[i] = -m_Vec[i] * 0.5;
		}
	}
	m_Value = CalValue();
}
// 每个粒子用相同的数组r1、r2
void Particle::Move(double chi, double c1, double c2, const double* r1, const double* r2, const double *gb) {
	if (gb == nullptr) gb = m_Pos_best_local;
	for (int i = 0; i < m_Size; i++) {
		m_Vec[i] = chi * (m_Vec[i] + c1 * r1[i] * (m_Pos_best[i] - m_Pos[i]) + c2 * r2[i] * (gb[i] - m_Pos[i]));
		m_Pos[i] += m_Vec[i];
		// 超出范围的处理
		if (m_Pos[i] < m_min[i]) {
			m_Pos[i] = m_min[i];
			m_Vec[i] = -m_Vec[i] * 0.5;
		}
		if (m_Pos[i] > m_max[i]) {
			m_Pos[i] = m_max[i];
			m_Vec[i] = -m_Vec[i] * 0.5;
		}
	}
	m_Value = CalValue();
}
// 每个粒子用不同的数组r1、r2
void Particle::Move(double chi, double c1, double c2, const double *gb) {
	if (gb == nullptr) gb = m_Pos_best_local;
	for (int i = 0; i < m_Size; i++) {
		double r1 = rand() / (double)(RAND_MAX);
		double r2 = rand() / (double)(RAND_MAX);
		m_Vec[i] = chi * (m_Vec[i] + c1 * r1 * (m_Pos_best[i] - m_Pos[i]) + c2 * r2 * (gb[i] - m_Pos[i]));
		m_Pos[i] += m_Vec[i];
		// 超出范围的处理
		if (m_Pos[i] < m_min[i]) {
			m_Pos[i] = m_min[i];
			m_Vec[i] = -m_Vec[i] * 0.5;
		}
		if (m_Pos[i] > m_max[i]) {
			m_Pos[i] = m_max[i];
			m_Vec[i] = -m_Vec[i] * 0.5;
		}
	}
	m_Value = CalValue();
}

/**
 * @brief 属性set方法
 *   为m_Pos_best_local和m_Value_best_local赋值
 * @param lb   局部最优位置
 * @param lbv  局部最优值
 */
void Particle::SetLocalBest(const double* lb, double lbv) {
	for (int i = 0; i < m_Size; i++) {
		m_Pos_best_local[i] = lb[i];
	}
	m_Value_best_local = lbv;
}

/**
 * @brief m_Value_best_local属性get方法
 * @return
 */
double Particle::GetLocalBestValue() const {
	return m_Value_best_local;
}

/**
 * @brief m_Value_best属性get方法
 * @return
 */
double Particle::GetSelfBestValue() const {
	return m_Value_best;
}

/**
 * @brief m_Pos_best属性get方法
 * @return 常量指针，不可修改其内容
 */
const double* Particle::GetSelfBestPos() const {
	return m_Pos_best;
}

ostream& operator<<(ostream &out, Particle &par) {
	out << setw(12) << "x: ";
	for (int i = 0; i < par.m_Size; i++) {
		out << par.m_Pos[i] << " ";
	}
	out << endl;
	out << setw(12) << "v: ";
	for (int i = 0; i < par.m_Size; i++) {
		out << par.m_Vec[i] << " ";
	}
	out << endl;
	out << setw(12) << "value: " << par.m_Value << endl;
	out << setw(12) << "x_best: ";
	for (int i = 0; i < par.m_Size; i++) {
		out << par.m_Pos_best[i] << " ";
	}
	out << endl;
	out << setw(12) << "bset value: " << par.m_Value << endl;
	return out;
}