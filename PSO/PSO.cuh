/***************************************************************
*  本文件包含粒子群算法类的声明，是本程序核心类                        *
*  @author   ZhRiT                                             *
*  @email    buaazhangrit@163.com                              *
*  @version  1.0.0                                             *
*  @date     2018-09-21                                        *
***************************************************************/
#pragma once

#include "pch.h"
#include "Benchmark.h"
#include <iostream>
#include <map>
#include "Particle.h"
#include <cuda_runtime.h>

using namespace std;

/**
 * @brief 粒子群优化算法类
 * 包含粒子群算法各个模块的操作和需要的属性（类似一个状态机）
 */
class PSO {
public:
	/*----- 核心功能 -----*/
	PSO() {
		m_controller[string("model")] = PSOMODEL::TEST;
		m_controller[string("useResample")] = PSOSTATUS_RESA::CLOSE;
		m_controller[string("resampleMethod")] = PSOFUNC_RESA::RESA_STAN;
		m_controller[string("topology")] = PSOFUNC_TOPO::GLOBAL;
		m_controller[string("minmax")] = PSOMINMAX::MIN;
		m_controller[string("parallel")] = PSODEVICE::CPU;
	};
	virtual ~PSO() {
		delete[] m_result_value_ite;
		delete[] m_result_objValue_ite;
		delete[] m_result_pos;
		delete[] m_vMax;
		m_objFun = nullptr;
		m_Particles = nullptr;
		m_min = nullptr; 
		m_max = nullptr;
		m_result_value_ite = nullptr;
		m_result_objValue_ite = nullptr;
		m_result_pos = nullptr;
		m_vMax = nullptr;
	};

	/**
	 * 枚举定义
	 */
	//运行模式
	typedef enum {
		TEST = 0,
		SOLVE,
	} PSOMODEL;
	//是否重采样
	typedef enum {
		CLOSE = 0,
		OPEN,
	} PSOSTATUS_RESA;
	//重采样方法
	typedef enum {
		RESA_STAN = 0,
		RESA_MULT,
		RESA_STRA,
		RESA_SYST,
		RESA_RESI,
		RESA_RESISYST,
	} PSOFUNC_RESA;
	//拓扑结构
	typedef enum {
		GLOBAL = 0,
		RING,
		RANDON,
	} PSOFUNC_TOPO;
	//最大值还是最小值
	typedef enum {
		MIN = 0,
		MAX,
	} PSOMINMAX;
	//运行设备
	typedef enum {
		CPU = 0,
		GPU,
	} PSODEVICE;

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
	void Initialize(double(*obj)(double*, int), int d, double *min, double *max, int n = 30, int t = 100, double opt = 0.0, double prec = 1e-6);

	/**
	 * @brief 启动算法，完成算法主要逻辑
	 */
	void Run();

	/**
	 * @brief 算法结果输出到控制台
	 */
	void Output() const;

	/**
	 * @brief 算法结果绘图
	 */
	void Draw() const;

	/**
	 * @brief 逐个设置算法控制器属性
	 * @param attr   控制器名称
	 * @param value  控制器的值
	 */
	void SetOption(const char*, int);

public:
	/*----- 属性存取器 -----*/
	void SetTMax(int);
	int GetTMax() const;

	void SetN(int);
	int GetN(int) const;

	void SetD(int);
	int GetD() const;

	void SetObjFun(double(*obj)(double*, int));

	void SetMin(double*);
	double *GetMin() const;

	void SetMax(double*);
	double *GetMax() const;

	void SetvMax(double*);
	double *GetVMax() const;

	void SetOptTheo(double);
	double GetOptTheo() const;

	void SetPrec(double);
	double GetPrec() const;

	double GetTimeCost() const;

	int GetTAct() const;

	int GetTObj() const;

	Status GetStatus() const;

	double *GetResultValueIte() const;
	double GetResultValue() const;
	double *GetResultPos() const;
	double *GetResultObjValueIte() const;
	double GetResultObjValue() const;

private:
	/**
	 * @brief 在CPU上运行
	 */
	void Run_CPU();

	/**
	 * @brief 在GPU上运行
	 */
	void Run_GPU();

	/**
	 * @brief 更新全局最优
	 * @param t_current  当前迭代次数
	 */
	void _UpdateGlobalBest(int);

	/**
	 * @brief 更新局部最优
	 */
	void _UpdateLocalBest();

	/**
	 * @brief 重采样
	 * @param 当前迭代次数
	 */
	void _Resample(int);

	/**
	 * @brief 标准重采样
	 * @param 当前迭代次数
	 */
	void _StanResample(int);

	/**
	 * @brief 多项式重采样
	 * @param 当前迭代次数
	 */
	void _MultResample(int);

	/**
	 * @brief 分层重采样
	 * @param 当前迭代次数
	 */
	void _StraResample(int);

	/**
	 * @brief 系统重采样
	 * @param 当前迭代次数
	 */
	void _SystResample(int);

	/**
	 * @brief 残差重采样
	 * @param 当前迭代次数
	 */
	void _ResiResample(int);

	/**
	 * @brief 残差系统重采样
	 * @param 当前迭代次数
	 */
	void _ResiSystResample(int);

	/**
	 * @brief 获取每个粒子的权重
	 * @param w 指向权重数组的指针
	 */
	void _GetWeight(double* w);

	/**
	 * @brief 随机重采样数组排序
	 * @param selected 代表采样关系的指针
	 */
	void _SelectSort(int* selected, int* selected_sort);

	/**
	 * @brief 根据重采样结果数组更新粒子
	 * @param s 重采样结果数组
	 * @param t 当前迭代次数
	 */
	void _UpdateParticles(const int* s, int t);

	/**
	 * @brief  重采样是，把另一个粒子的数据拷贝过来
	 * @param  拷贝粒子的指针
	 * @param  被拷贝的粒子的指针
	 * @param  当前迭代次数
	 */
	void _Copy4Resample(Particle*, const Particle*, int);

private:
	/*----- 基本信息 -----*/
	int m_tMax { 100 };                           // 最大迭代次数
	int m_number { 30 };                          // 粒子个数
	int m_d { 0 };                                // 问题维数
	double(*m_objFun)(double*, int) { nullptr };  // 目标函数指针
	Particle *m_Particles{ nullptr };             // 粒子群
	double *m_min{ nullptr };                     // 下界
	double *m_max{ nullptr };                     // 上界
	double *m_vMax{ nullptr };                    // 最大速度

	/*----- 结果 -----*/
	double *m_result_value_ite;                   // 每一次迭代的最优值(适应度)
	double *m_result_objValue_ite;                // 每一次迭代的最优值(目标函数)
	vector<ResultInfo> m_result_ct_ite;           // 每一次迭代的约束值
	double m_result_value;                        // 全局最优值（适应度结果）
	double m_result_objValue;                     // 全局最优值（目标函数结果）
	ResultInfo m_result_ct;                       // 优化结果的约束值
	double *m_result_pos;                         // 全局最优位置（结果）
	int m_t_act;                                  // 实际迭代次数
	double m_time_cost;                           // 消耗的时间
	int m_t_obj;                                  // 目标函数执行次数
	/**
	 * @brief 算法结束状态
	 *  -INPUTERROR 输入错误
	 *  -SUCCESS    优化成功（测试模式，达到精度）
	 *  -FAIL       优化失败（测试模式，未达到精度）
	 *  -OK         优化完成（求解模式，达到最大迭代次数）
	 */
	Status m_status;

	// 终止条件相关
	double m_precision{ 1e-6 };                   // 精度，测试环境下用到
	double m_opt_theo{ 0.0 };                     // 理论最优值，测试模式下用到

	/**
	 * @brief 算法状态控制器
	 * @key model   运行模式：
	 *   -0 测试模式，用标准测试呢函数测试算法性能
	 *   -1 求解模式，求解具体问题
	 * @key useResample   是否开启重采
	 *   -0 关闭
	 *   -1 开启
	 * @key resampleMethod   重采样方式
	 *   -0 标准重采样
	 *   -1 多项式重采样
	 *   -2 分层重采样
	 *   -3 系统重采样
	 *   -4 残差重采样
	 * @key topology   拓扑方式 
	 *   -0 全局拓扑
	 *   -1 环形拓扑
	 *   -2 随机拓扑
	 * @key minmax   优化方式
	 *   -0 最小值优化
	 *   -1 最大值优化
	 * @key parallel 是否利用GPU并行
	 *   -0 CPU
	 *   -1 GPU
	 */
	map<string, int> m_controller;

};

/**
 * GPU-PSO的核函数
 */
namespace psokernel {
	/**
	 * @breif 随机初始化粒子
	 * @param xx      位置
	 * @param vx      速度
	 * @param pbestx  个体最优位置
	 * @param gbest   全局最优标记
	 * @param d       维数
	 * @param n       粒子个数
	 * @param min     位置下界
	 * @param max     位置上界
	 */
	__global__ void InitParticles(double *xx, double *vx, double *pbestx, int *gbest, int d, int n, double *min, double *max);

	/**
	 * @breif 求目标函数值
	 * @param xx      位置
	 * @param value   目标函数值
	 * @param d       维数
	 * @param n       粒子个数
	 * @param funcIdx 目标函数序号
	 */
	__global__ void GetFitness(double *xx, double *value, int d, int n, int funcIdx);

	/**
	 * @breif 更新个体最优
	 * @param xx      位置
	 * @param value   目标函数值
	 * @param pbestx  个体最优位置
	 * @param pbest   个体最优值
	 * @param d       维数
	 * @param n       粒子个数
	 * @param iter    当前迭代次数
	 */
	__global__ void UpdatePbest(double *xx, double *value, double *pbestx, double *pbest, int d, int n, int iter);

	/**
	 * @breif 更新个体最优
	 * @param gbest      全局最优标记
	 * @param pbest      个体最优值
	 * @param d          维数
	 * @param n          粒子个数
	 * @param threadNum  线程数 大于等于粒子个数的最小2的幂
	 */
	__global__ void UpdateGbest(int *gbest, double *pbest, int d, int n, int threadNum);

	/**
	 * @breif 更新粒子位置
	 * @param xx      位置
	 * @param vx      速度
	 * @param pbestx  个体最优位置
	 * @param gbest   全局最优标记
	 * @param d       维数
	 * @param n       粒子个数
	 * @param min     位置下界
	 * @param max     位置上界
	 */
	__global__ void UpdateParticles(double *xx, double *vx, double *pbestx, int *gbest, int d, int n, double *min, double *max);

}
