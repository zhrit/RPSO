/***************************************************************
*  ���ļ�����PSO�㷨���ܲ�����غ���                                *
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
 * PSO�㷨������ز���
 */
struct PerfInfo {
	int PopulationSize; // ���Ӹ���
	int Iterations;     // ��ƽ������������
	int Iterations_S;   // ��ƽ�����ɹ������ĵ�������
	double Time;        // ��ƽ��������ʱ��
	double Time_S;      // ��ƽ�����ɹ�����������ʱ��
	double Time_O;      // ��ƽ�����ɹ�������Ŀ�꺯��ִ�д���
	double SuccessRate; // �ɹ���
	int Repeat;         // ִ�д���
	int dem;            // ά��
};

/**
 * @brief ���ܲ�����
 */
class Performance {
public:
	Performance();
	~Performance() {}

	/**
	 * @breif ����ά�ȣ�����������������������
	 * @ param serial Ŀ�꺯�����
	 * @ param d      ά��
	 * @ param min    �Ա����½�
	 * @ param min    �Ա����Ͻ�
	 * @ param n      ���Ӹ���
	 * @ param repeat �ظ�����
	 * @ param tmax   ����������
	 */
	PerfInfo SingleTest(int serial, int d, double *min, double *max, int n, int repeat = 30, int tmax = 3000);

	/**
	 * @breif ����ά�ȣ��������������ܵ�Ӱ��
	 * @ param serial   Ŀ�꺯�����
	 * @ param d        ά��
	 * @ param min      �Ա����½�
	 * @ param min      �Ա����Ͻ�
	 * @ param repeat   �ظ�����
	 * @ param tmax     ����������
	 * @ param fileName �������ݵ��ļ���
	 * @ param n_l      ����������Χ�½�
	 * @ param n_u      ����������Χ�Ͻ�
	 */
	void EffectOfPopulation(int serial, int d, double *min, double *max, int repeat = 30, int tmax = 3000, string fileName = "", int n_l = 3, int n_u = 30);

	/**
	 * @breif ��ͬά��ά�ȣ��������������ܵ�Ӱ��
	 * @ param serial   Ŀ�꺯�����
	 * @ param min      �Ա����½�
	 * @ param min      �Ա����Ͻ�
	 * @ param repeat   �ظ�����
	 * @ param tmax     ����������
	 * @ param fileName �������ݵ��ļ���
	 */
	void EffectOfPopulation_D(int serial, double *min, double *max, int repeat = 30, int tmax = 3000, string fileName = "");

	/**
	 * @brief �ѽ��д��excel�ļ�
	 * @param infos ���н��
	 */
	void WriteToCSV(const vector<PerfInfo> &infos, const string fileName);

private:
	// ���Ժ�����
	double (*FuncPool[21])(double*, int) = { Benchmark::Sphere, Benchmark::Schwefel1_2, 
		Benchmark::Schwefel2_22, Benchmark::Schwefel2_21, Benchmark::Generalized, Benchmark::Step, Benchmark::Quartic,
		Benchmark::Generalized_Schwefel2_26, Benchmark::Generalized_Rastrigin, Benchmark::Ackley,
		Benchmark::Generalized_Griewank, Benchmark::Generalized_Penalized_1, Benchmark::Generalized_Penalized_2,
		Benchmark::Shekel_Foxholes, Benchmark::Kowalik, Benchmark::Camel_Back, Benchmark::Branin,
		Benchmark::Goldstein_Price, Benchmark::Neumaire, Benchmark::Salomon, Benchmark::Alpine };
	// ���Ժ�����������ֵ
	double FuncOptimals[21] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -418.9828872724338, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.998004, 0.0003075, -1.0316285, 0.397887364, 3.0,
		0.0, 0.0, 0.0};
	// �������Ժ�����Ӧ�ľ���
	double FuncPrec[21] = { 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6,
		1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6 };
	// ���Ժ��������ռ��С
	double FuncSpaceSize[21] = { 100, 100, 10, 100, 30, 100, 1.28, 500, 5.12,
		32, 600, 50, 50, 65.536, 5, 5, 15, 2, 1, 100, 10
	};

	bool HasSetRandomSeed{ false }; // �Ƿ����ù��������

	/**
	 * �������������
	 */
	void SetRandomSeed();
};