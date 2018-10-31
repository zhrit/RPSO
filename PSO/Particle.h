/***************************************************************
*  本文件包含粒子类的声明                                          *
*  @author   ZhRiT                                             *
*  @email    buaazhangrit@163.com                              *
*  @version  1.0.0                                             *
*  @date     2018-09-21                                        *
***************************************************************/
#pragma once

#include "Benchmark.h"
#include <iostream>
#include <map>
using namespace std;

/**
 * @brief 粒子类
 * 包含粒子群算法中每个粒子应该具备的数据成员和方法
 */
class Particle {
	friend ostream& operator<<(ostream &out, Particle &par);
public:
	Particle();
	virtual ~Particle();

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
	void Initialize(int d, double* min, double* max, double(*objFun)(double*, int));

	/**
	 * @brief 随机产生粒子的位置和速度
	 */
	void Random();

	/**
	 * @brief 给个体最优和局部最优赋初值
	 *   个体最优在速度和位置更新中用到
	 *   局部最优在环形拓扑和随机拓扑中用到
	 */
	void InitBest();

	/**
	 * @brief 计算当前位置坐标对应的目标函数值
	 */
	double CalValue() const;

	/**
	 * @brief 更新个体最优和局部最优
	 * @param index min_max优化标志
	 *   -0 min优化
	 *   -1 max优化
	 */
	void UpdateBest(int index = 0);

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
	void Move(double, double, double, double, double, const double*);
	// 每个粒子用相同的数组r1、r2
	void Move(double, double, double, const double*, const double*, const double*);
	// 每个粒子用不同的数组r1、r2
	void Move(double, double, double, const double*);

	/**
	 * @brief 属性set方法
	 *   为m_Pos_best_local和m_Value_best_local赋值
	 * @param lb   局部最优位置
	 * @param lbv  局部最优值
	 */
	void SetLocalBest(const double*, double);

	/**
	 * @brief m_Value_best_local属性get方法
	 * @return
	 */
	double GetLocalBestValue() const;

	/**
	 * @brief m_Value_best属性get方法
	 * @return
	 */
	double GetSelfBestValue() const;

	/**
	 * @brief m_Pos_best属性get方法
	 * @return 常量指针，不可修改其内容
	 */
	const double* GetSelfBestPos() const;

public:
	double *m_Pos;                              // 当前位置
	double *m_Vec;                              // 当前速度
	double m_Value;                             // 当前位置对应的目标函数值
	double *m_min;                              // 位置下界
	double *m_max;                              // 位置上界
	int m_Size;                                 // 维数
	bool isNull{ true };                        // 是否为空（是否经过初始化）

	double *m_Pos_best;                         // 个体最优位置
	double m_Value_best;                        // 个体最优值

	double *m_Pos_best_local;                   // 局部最优位置
	double m_Value_best_local;                  // 局部最优值

	double(*m_objFun)(double*, int) { nullptr };// 函数指针

	static int m_Tmax;                          // 最大迭代次数
};