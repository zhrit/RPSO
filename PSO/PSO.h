/***************************************************************
*  ���ļ���������Ⱥ�㷨����������Ǳ����������                        *
*  @author   ZhRiT                                             *
*  @email    buaazhangrit@163.com                              *
*  @version  1.0.0                                             *
*  @date     2018-09-21                                        *
***************************************************************/
#pragma once

#include "Benchmark.h"
#include <iostream>
#include <map>
#include "Particle.h"

using namespace std;

/**
 * @brief ����Ⱥ�Ż��㷨��
 * ��������Ⱥ�㷨����ģ��Ĳ�������Ҫ�����ԣ�����һ��״̬����
 */
class PSO {
public:
	/*----- ���Ĺ��� -----*/
	PSO() {
		m_controller[string("model")] = 0;         //ģʽ��0-����ģʽ��1-���ģʽ
		m_controller[string("useResample")] = 0;   //�Ƿ����ز�����0-�رգ�1-����
		m_controller[string("resampleMethod")] = 0;//�ز�����ʽ
		m_controller[string("topology")] = 0;      //���˷�ʽ 0-ȫ������ 1-�������� 2-�������
		m_controller[string("minmax")] = 0;        //0-��Сֵ��1-���ֵ
		m_controller[string("parallel")] = 0;      //0-CPU��1-GPU
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

	/*----- ��� -----*/
	double *m_result_value_ite;                   // ÿһ�ε���������ֵ
	double m_result_value;                        // ȫ������ֵ�������
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
