/***************************************************************
*  ���ļ���������Ⱥ�㷨����������Ǳ����������                        *
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
 * @brief ����Ⱥ�Ż��㷨��
 * ��������Ⱥ�㷨����ģ��Ĳ�������Ҫ�����ԣ�����һ��״̬����
 */
class PSO {
public:
	/*----- ���Ĺ��� -----*/
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
	 * ö�ٶ���
	 */
	//����ģʽ
	typedef enum {
		TEST = 0,
		SOLVE,
	} PSOMODEL;
	//�Ƿ��ز���
	typedef enum {
		CLOSE = 0,
		OPEN,
	} PSOSTATUS_RESA;
	//�ز�������
	typedef enum {
		RESA_STAN = 0,
		RESA_MULT,
		RESA_STRA,
		RESA_SYST,
		RESA_RESI,
		RESA_RESISYST,
	} PSOFUNC_RESA;
	//���˽ṹ
	typedef enum {
		GLOBAL = 0,
		RING,
		RANDON,
	} PSOFUNC_TOPO;
	//���ֵ������Сֵ
	typedef enum {
		MIN = 0,
		MAX,
	} PSOMINMAX;
	//�����豸
	typedef enum {
		CPU = 0,
		GPU,
	} PSODEVICE;

	/**
	 * @brief �㷨��ʼ��
	 *   �����Ҫ�Ĳ���
	 * @param obj  Ŀ�꺯��
	 * @param d    ά��
	 * @param min  �½�
	 * @param max  �Ͻ�
	 * @param n    ���Ӹ���
	 * @param t    ����������
	 * @param opt  ��������ֵ������ģʽ���õ�
	 * @param prec ���ȣ�����ģʽ���õ�
	 */
	void Initialize(double(*obj)(double*, int), int d, double *min, double *max, int n = 30, int t = 100, double opt = 0.0, double prec = 1e-6);

	/**
	 * @brief �����㷨������㷨��Ҫ�߼�
	 */
	void Run();

	/**
	 * @brief �㷨������������̨
	 */
	void Output() const;

	/**
	 * @brief �㷨�����ͼ
	 */
	void Draw() const;

	/**
	 * @brief ��������㷨����������
	 * @param attr   ����������
	 * @param value  ��������ֵ
	 */
	void SetOption(const char*, int);

public:
	/*----- ���Դ�ȡ�� -----*/
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
	 * @brief ��CPU������
	 */
	void Run_CPU();

	/**
	 * @brief ��GPU������
	 */
	void Run_GPU();

	/**
	 * @brief ����ȫ������
	 * @param t_current  ��ǰ��������
	 */
	void _UpdateGlobalBest(int);

	/**
	 * @brief ���¾ֲ�����
	 */
	void _UpdateLocalBest();

	/**
	 * @brief �ز���
	 * @param ��ǰ��������
	 */
	void _Resample(int);

	/**
	 * @brief ��׼�ز���
	 * @param ��ǰ��������
	 */
	void _StanResample(int);

	/**
	 * @brief ����ʽ�ز���
	 * @param ��ǰ��������
	 */
	void _MultResample(int);

	/**
	 * @brief �ֲ��ز���
	 * @param ��ǰ��������
	 */
	void _StraResample(int);

	/**
	 * @brief ϵͳ�ز���
	 * @param ��ǰ��������
	 */
	void _SystResample(int);

	/**
	 * @brief �в��ز���
	 * @param ��ǰ��������
	 */
	void _ResiResample(int);

	/**
	 * @brief �в�ϵͳ�ز���
	 * @param ��ǰ��������
	 */
	void _ResiSystResample(int);

	/**
	 * @brief ��ȡÿ�����ӵ�Ȩ��
	 * @param w ָ��Ȩ�������ָ��
	 */
	void _GetWeight(double* w);

	/**
	 * @brief ����ز�����������
	 * @param selected ���������ϵ��ָ��
	 */
	void _SelectSort(int* selected, int* selected_sort);

	/**
	 * @brief �����ز�����������������
	 * @param s �ز����������
	 * @param t ��ǰ��������
	 */
	void _UpdateParticles(const int* s, int t);

	/**
	 * @brief  �ز����ǣ�����һ�����ӵ����ݿ�������
	 * @param  �������ӵ�ָ��
	 * @param  �����������ӵ�ָ��
	 * @param  ��ǰ��������
	 */
	void _Copy4Resample(Particle*, const Particle*, int);

private:
	/*----- ������Ϣ -----*/
	int m_tMax { 100 };                           // ����������
	int m_number { 30 };                          // ���Ӹ���
	int m_d { 0 };                                // ����ά��
	double(*m_objFun)(double*, int) { nullptr };  // Ŀ�꺯��ָ��
	Particle *m_Particles{ nullptr };             // ����Ⱥ
	double *m_min{ nullptr };                     // �½�
	double *m_max{ nullptr };                     // �Ͻ�
	double *m_vMax{ nullptr };                    // ����ٶ�

	/*----- ��� -----*/
	double *m_result_value_ite;                   // ÿһ�ε���������ֵ(��Ӧ��)
	double *m_result_objValue_ite;                // ÿһ�ε���������ֵ(Ŀ�꺯��)
	double m_result_value;                        // ȫ������ֵ����Ӧ�Ƚ����
	double m_result_objValue;                     // ȫ������ֵ��Ŀ�꺯�������
	double *m_result_pos;                         // ȫ������λ�ã������
	int m_t_act;                                  // ʵ�ʵ�������
	double m_time_cost;                           // ���ĵ�ʱ��
	int m_t_obj;                                  // Ŀ�꺯��ִ�д���
	/**
	 * @brief �㷨����״̬
	 *  -INPUTERROR �������
	 *  -SUCCESS    �Ż��ɹ�������ģʽ���ﵽ���ȣ�
	 *  -FAIL       �Ż�ʧ�ܣ�����ģʽ��δ�ﵽ���ȣ�
	 *  -OK         �Ż���ɣ����ģʽ���ﵽ������������
	 */
	Status m_status;

	// ��ֹ�������
	double m_precision{ 1e-6 };                   // ���ȣ����Ի������õ�
	double m_opt_theo{ 0.0 };                     // ��������ֵ������ģʽ���õ�

	/**
	 * @brief �㷨״̬������
	 * @key model   ����ģʽ��
	 *   -0 ����ģʽ���ñ�׼�����غ��������㷨����
	 *   -1 ���ģʽ������������
	 * @key useResample   �Ƿ����ز�
	 *   -0 �ر�
	 *   -1 ����
	 * @key resampleMethod   �ز�����ʽ
	 *   -0 ��׼�ز���
	 *   -1 ����ʽ�ز���
	 *   -2 �ֲ��ز���
	 *   -3 ϵͳ�ز���
	 *   -4 �в��ز���
	 * @key topology   ���˷�ʽ 
	 *   -0 ȫ������
	 *   -1 ��������
	 *   -2 �������
	 * @key minmax   �Ż���ʽ
	 *   -0 ��Сֵ�Ż�
	 *   -1 ���ֵ�Ż�
	 * @key parallel �Ƿ�����GPU����
	 *   -0 CPU
	 *   -1 GPU
	 */
	map<string, int> m_controller;

};

/**
 * GPU-PSO�ĺ˺���
 */
namespace psokernel {
	/**
	 * @breif �����ʼ������
	 * @param xx      λ��
	 * @param vx      �ٶ�
	 * @param pbestx  ��������λ��
	 * @param gbest   ȫ�����ű��
	 * @param d       ά��
	 * @param n       ���Ӹ���
	 * @param min     λ���½�
	 * @param max     λ���Ͻ�
	 */
	__global__ void InitParticles(double *xx, double *vx, double *pbestx, int *gbest, int d, int n, double *min, double *max);

	/**
	 * @breif ��Ŀ�꺯��ֵ
	 * @param xx      λ��
	 * @param value   Ŀ�꺯��ֵ
	 * @param d       ά��
	 * @param n       ���Ӹ���
	 * @param funcIdx Ŀ�꺯�����
	 */
	__global__ void GetFitness(double *xx, double *value, int d, int n, int funcIdx);

	/**
	 * @breif ���¸�������
	 * @param xx      λ��
	 * @param value   Ŀ�꺯��ֵ
	 * @param pbestx  ��������λ��
	 * @param pbest   ��������ֵ
	 * @param d       ά��
	 * @param n       ���Ӹ���
	 * @param iter    ��ǰ��������
	 */
	__global__ void UpdatePbest(double *xx, double *value, double *pbestx, double *pbest, int d, int n, int iter);

	/**
	 * @breif ���¸�������
	 * @param gbest      ȫ�����ű��
	 * @param pbest      ��������ֵ
	 * @param d          ά��
	 * @param n          ���Ӹ���
	 * @param threadNum  �߳��� ���ڵ������Ӹ�������С2����
	 */
	__global__ void UpdateGbest(int *gbest, double *pbest, int d, int n, int threadNum);

	/**
	 * @breif ��������λ��
	 * @param xx      λ��
	 * @param vx      �ٶ�
	 * @param pbestx  ��������λ��
	 * @param gbest   ȫ�����ű��
	 * @param d       ά��
	 * @param n       ���Ӹ���
	 * @param min     λ���½�
	 * @param max     λ���Ͻ�
	 */
	__global__ void UpdateParticles(double *xx, double *vx, double *pbestx, int *gbest, int d, int n, double *min, double *max);

}
