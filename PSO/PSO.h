/***************************************************************
*  本文件包含粒子类和粒子群算法类的声明，是本程序核心类                  *
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


/**
 * @brief 粒子群优化算法类
 * 包含粒子群算法各个模块的操作和需要的属性（类似一个状态机）
 */
class PSO {
public:
	/*----- 核心功能 -----*/
	PSO() {
		m_controller[string("model")] = 0;         //模式：0-测试模式，1-求解模式
		m_controller[string("useResample")] = 0;   //是否开启重采样：0-关闭，1-开启
		m_controller[string("resampleMethod")] = 0;//重采样方式
		m_controller[string("topology")] = 0;      //拓扑方式 0-全局拓扑 1-环形拓扑 2-随机拓扑
		m_controller[string("minmax")] = 0;        //0-最小值，1-最大值
		m_controller[string("parallel")] = 0;      //0-CPU，1-GPU
	};
	virtual ~PSO() {
		delete[] m_result_value_ite;
		delete[] m_result_pos;
		m_objFun = nullptr;
		m_Particles = nullptr;
		m_min = nullptr; 
		m_max = nullptr;
		m_result_value_ite = nullptr;
		m_result_pos = nullptr;
	};

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

	void SetOptTheo(double);
	double GetOptTheo() const;

	void SetPrec(double);
	double GetPrec() const;

	double GetTimeCost() const;

	int GetTAct() const;

	int GetTObj() const;

	Status GetStatus() const;

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

	/*----- 结果 -----*/
	double *m_result_value_ite;                   // 每一次迭代的最优值
	double m_result_value;                        // 全局最优值（结果）
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
