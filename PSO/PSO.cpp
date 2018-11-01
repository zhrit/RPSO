/***************************************************************
*  ���ļ���������Ⱥ�㷨��Ķ��壬�Ǳ����������                        *
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

/*----------------------------------------------------------����Ⱥ�Ż��㷨�ඨ��--------------------------------------------------------------*/

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
 * @brief �����㷨������㷨��Ҫ�߼�
 */
void PSO::Run() {
	/********** Ԥ���� **********/
	// Ŀ�꺯��ִ�д�������Ϊ0
	Benchmark::T = 0;
	m_t_obj = 0;
	/********** ������� **********/
	if (m_d == 0) {
		cout << "��ͨ��Initialize()������SetD()���������Ż������ά�ȣ�" << endl;
		m_status = INPUTERROR;
	}
	else {}
	if (m_objFun == nullptr) {
		cout << "��ͨ��Initialize()������SetObjFun()���������Ż������Ŀ�꺯����" << endl;
		m_status = INPUTERROR;
	}
	if (m_min == nullptr || m_max == nullptr) {
		cout << "��ͨ��Initialize()������SetMin()��SetMax()���������Ż������Ŀ�꺯����" << endl;
		m_status = INPUTERROR;
	}

	/********** �㷨״̬��� **********/
	string topologyStr[3] = { "ȫ������", "��������", "�������" };
	cout << "=====�㷨������=====" << endl;
	cout << "=====" << (m_controller[string("model")] ? "���ģʽ" : "����ģʽ") << "==";
	cout << topologyStr[m_controller[string("topology")]] << "==";
	cout << (m_controller[string("minmax")] ? "���ֵ����" : "��Сֵ����") << "==";
	cout << (m_controller[string("parallel")] ? "GPU" : "CPU") << "=====" << endl;
	cout << "������������" << m_tMax << "��  ���Ӹ�����" << m_number << "��  ����ά�ȣ�" << m_d << "��" << endl;

	double beginTime = GetTickCount();
	if (m_controller[string("parallel")] == 0) {
		Run_CPU();
	}
	else {
		Run_GPU();
	}

	double endTime = GetTickCount();
	m_time_cost = endTime - beginTime;

	cout << "=====������=====" << endl;
	Output();
}

/**
 * @brief ��CPU������
 */
void PSO::Run_CPU() {
	// ������ʼ��
	const double c1 = 2.05, c2 = 2.05;
	const double phi = c1 + c2;
	const double chi = 2.0 / abs(2.0 - phi - sqrt(phi * phi - 4.0 * phi));

	// ��ʼ������
	//double* r11 = new double[m_d];
	//double* r22 = new double[m_d];
	//cout << "=====��ʼ�����ӣ�=====" << endl;
	m_Particles = new Particle[m_number];
	for (int i = 0; i < m_number; i++) {
		(m_Particles + i)->Initialize(m_d, m_min, m_max, m_objFun);
		// cout << *(m_Particles + i) << endl;
	}
	// ��ѭ��
	for (int t = 0; t <= m_tMax; t++) {
		// ���¸�������λ�ú͸�������ֵ
		for (int i = 0; i < m_number; i++) {
			(m_Particles + i)->UpdateBest(m_controller["minmax"]);
		}
		// ����ȫ������λ�ú�ȫ������ֵ
		_UpdateGlobalBest(t);
		// �ж��Ƿ�ﵽ���Ⱥ��˳�ѭ��������
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
		// �ز���
		if (m_controller[string("useResample")] == 1) {
			_Resample(t);
		}
		// �ٶȸ���
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
	// �����ڴ�
	delete[] m_Particles;
	m_Particles = nullptr;
	//delete[] r11;
	//r11 = nullptr;
	//delete[] r22;
	//r22 = nullptr;
}

/**
 * @brief �㷨������������̨
 */
void PSO::Output() const {
	cout << "=====�����=====" << endl;
	cout << "���ŵ㣺";
	for (int i = 0; i < m_d; i++) {
		cout << m_result_pos[i] << ", ";
	}
	cout << endl << "����ֵ��" << m_result_value << endl;
	cout << "ʵ�ʵ���������" << m_t_act << endl;
	cout << "ʵ������ʱ�䣺" << m_time_cost << endl;
	cout << "Ŀ�꺯�����д�����" << Benchmark::T << endl;
	cout << "���״ֵ̬��" << m_status << endl;
}

/**
 * @brief �㷨�����ͼ
 */
void PSO::Draw() const {
	//TODO
}

/**
 * @brief ��������㷨����������
 * @param attr   ����������
 * @param value  ��������ֵ
 */
void PSO::SetOption(const char* attr, int value) {
	if (m_controller.count(attr) == 0) {
		cout << "û�д����ԣ�" << endl;
		return;
	}
	m_controller[attr] = value;
}

/*----- ���Դ�ȡ�� -----*/
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

/*----- �ڲ����� -----*/

/**
 * @brief ����ȫ������
 * @param t_current  ��ǰ��������
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
 * @brief ���¾ֲ�����
 */
void PSO::_UpdateLocalBest() {
	if (m_controller[string("topology")] == 1) {
		// ��������
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
		// �������
		if (m_controller["minmax"] == 0) {
		}
		else {

		}
	}
	else {

	}
}

/**
 * @brief �ز���
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
 * @brief ��׼�ز���
 * @param ��ǰ��������
 */

void PSO::_StanResample(int t) {
	// ����Ȩ��
	double* weights = new double[m_number];
	_GetWeight(weights);

	// �ز��������ж�
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

	// Ȩ�صľ�ֵ
	double w_mean = 0.0;
	for (int i = 0; i < m_number; i++) {
		w_mean += weights[i];
	}
	w_mean = w_mean / double(m_number);

	// Ȩ�صķ���
	double w_var = 0.0;
	for (int i = 0; i < m_number; i++) {
		w_var += pow(weights[i] - w_mean, 2);
	}
	w_var /= double(m_number);

	// ��ֵ
	double w_t = w_mean - 2 * w_var;
	// Ȩ��С���ز�������������µ�����
	for (int i = 0; i < m_number; i++) {
		if (weights[i] < w_t) {
			//cout << i << ", ";
			(m_Particles + i)->Random();
			(m_Particles + i)->InitBest();
		}
	}
	//cout << endl;

	// �ͷ��ڴ�
	delete[] weights;
	weights = nullptr;
}

/**
 * @brief ����ʽ�ز���
 */
void PSO::_MultResample(int t) {
	// ����Ȩ��
	double* weights = new double[m_number];
	_GetWeight(weights);

	// �ز��������ж�
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

	// Ȩ��һ�����
	for (int i = 1; i < m_number; i++) {
		weights[i] += weights[i - 1];
	}

	// ȡ�����
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

	// ����selcted����ȡ�ز�������������ʾ
	_SelectSort(selected, selected_sort);

	// �����ز�����������������
	_UpdateParticles(selected_sort, t);


	// �ͷ��ڴ�
	delete[] selected;
	selected = nullptr;
	delete[] selected_sort;
	selected_sort = nullptr;
	delete[] weights;
	weights = nullptr;
}

/**
 * @brief �ֲ��ز���
 */
void PSO::_StraResample(int t) {
	// ����Ȩ��
	double* weights = new double[m_number];
	_GetWeight(weights);

	// �ز��������ж�
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

	// Ȩ��һ�����
	for (int i = 1; i < m_number; i++) {
		weights[i] += weights[i - 1];
	}

	// ȡ�����
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

	// ����selcted����ȡ�ز�������������ʾ
	_SelectSort(selected, selected_sort);

	// �����ز�����������������
	_UpdateParticles(selected_sort, t);


	// �ͷ��ڴ�
	delete[] selected;
	selected = nullptr;
	delete[] selected_sort;
	selected_sort = nullptr;
	delete[] weights;
	weights = nullptr;
}

/**
 * @brief ϵͳ�ز���
 */
void PSO::_SystResample(int t) {
	// ����Ȩ��
	double* weights = new double[m_number];
	_GetWeight(weights);

	// �ز��������ж�
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

	// Ȩ��һ�����
	for (int i = 1; i < m_number; i++) {
		weights[i] += weights[i - 1];
	}

	// ȡ�����
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

	// ����selcted����ȡ�ز�������������ʾ
	_SelectSort(selected, selected_sort);

	// �����ز�����������������
	_UpdateParticles(selected_sort, t);


	// �ͷ��ڴ�
	delete[] selected;
	selected = nullptr;
	delete[] selected_sort;
	selected_sort = nullptr;
	delete[] weights;
	weights = nullptr;
}

/**
 * @brief �в��ز���
 */
void PSO::_ResiResample(int t) {
	// ����Ȩ��
	double* weights = new double[m_number];
	_GetWeight(weights);

	// �ز��������ж�
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

	int *selected = new int[m_number]; // �ز�������

	int selectedIndex = 0; // selected�������λ�ñ��

	// Ȩ�ش���1/N�ĸ��ƣ����ƴ���Ϊfloor��N * w��
	for (int i = 0; i < m_number; i++) {
		double num4copy = floor(m_number * weights[i]);// ���㸴�ƴ���
		weights[i] = weights[i] - num4copy / double(m_number); // ��Ȩ��
		// ���selected����
		for (int j = 0; j < num4copy; j++) {
			selected[selectedIndex] = i;
			selectedIndex++;
		}
	}

	// ������Ƶ���������С����Ⱥ�����������ö���ʽ�ز�������ȡʣ�µ�����
	if (selectedIndex < m_number) {
		// ��Ȩ�ع�һ��
		for (int i = 0; i < m_number; i++) {
			weights[i] = weights[i] * double(m_number) / double(m_number - selectedIndex);
		}
		// ��Ȩ��һ�����
		for (int i = 1; i < m_number; i++) {
			weights[i] += weights[i - 1];
		}
		// ����ʽ�ز���
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

	// ����selcted����ȡ�ز�������������ʾ
	_SelectSort(selected, selected_sort);

	// �����ز�����������������
	_UpdateParticles(selected_sort, t);


	// �ͷ��ڴ�
	delete[] selected;
	selected = nullptr;
	delete[] selected_sort;
	selected_sort = nullptr;
	delete[] weights;
	weights = nullptr;
}

/**
 * @brief �в�ϵͳ�ز���
 * @param ��ǰ��������
 */
void PSO::_ResiSystResample(int t) {
	// ����Ȩ��
	double* weights = new double[m_number];
	_GetWeight(weights);

	// �ز��������ж�
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

	int *selected = new int[m_number]; // �ز�������

	int selectedIndex = 0; // selected�������λ�ñ��

	double u0 = rand() / double(RAND_MAX) / double(m_number);

	// Ȩ�ش���1/N�ĸ��ƣ����ƴ���Ϊfloor��N * w��
	for (int i = 0; i < m_number; i++) {
		double num4copy = floor(m_number * (weights[i] - u0)) + 1;// ���㸴�ƴ���
		u0 = u0 + num4copy / double(m_number) - weights[i];
		// ���selected����
		for (int j = 0; j < num4copy; j++) {
			selected[selectedIndex] = i;
			selectedIndex++;
		}
	}
	cout << selectedIndex << endl;

	int *selected_sort = new int[m_number];

	// ����selcted����ȡ�ز�������������ʾ
	_SelectSort(selected, selected_sort);

	// �����ز�����������������
	_UpdateParticles(selected_sort, t);


	// �ͷ��ڴ�
	delete[] selected;
	selected = nullptr;
	delete[] selected_sort;
	selected_sort = nullptr;
	delete[] weights;
	weights = nullptr;
}

/**
 * @brief ��ȡÿ�����ӵ�Ȩ��
 * @param w ָ��Ȩ�������ָ��
 */
void PSO::_GetWeight(double* w) {
	// ������ֵ�Ĳ��
	double sum = 0.0;
	for (int i = 0; i < m_number; i++) {
		w[i] = abs(m_Particles[i].m_Value - m_result_value);
		sum += w[i];
	}
	// ���ֵ����
	double mean = sum / double(m_number);
	double var = 0.0;
	for (int i = 0; i < m_number; i++) {
		var += pow(w[i] - mean, 2);
	}
	var /= double(m_number);
	// ��Ȩ��
	for (int i = 0; i < m_number; i++) {
		w[i] = 1.0 / sqrt(2 * PI * var) * exp(-w[i] * w[i] / (2 * var));
	}
	// Ȩ�ع�һ��
	sum = 0.0;
	for (int i = 0; i < m_number; i++) {
		sum += w[i];
	}
	for (int i = 0; i < m_number; i++) {
		w[i] /= sum;
	}
}

/**
 * @brief ����ز���
 * @param selected ���������ϵ��ָ��
 */
void PSO::_SelectSort(int* selected, int* selected_sort) {
	// seleted��ѡ���������ӣ�seleted_sort�Ƕ�ѡ�����ӵ�����Ϊ���Ǿ�������ԭ�е�˳��
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
 * @brief �����ز�����������������
 * @param s �ز����������
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
 * @brief  �ز���ʱ������һ�����ӵ����ݿ�������
 * @param  �������ӵ�ָ��
 * @param  �����������ӵ�ָ��
 * @param  ��ǰ��������
 */
void PSO::_Copy4Resample(Particle* p1, const Particle* p2, int t) {
	// ��ǰλ��-����
	for (int i = 0; i < m_d; i++) {
		p1->m_Pos[i] = p2->m_Pos[i];
		// p1->m_Vec[i] = p2->m_Vec[i];
	}
	// ��ǰλ�ö�Ӧ��Ŀ�꺯��ֵ-����
	p1->m_Value = p2->m_Value;

	// ��ǰ�ٶ�-�ϳ�
	for (int i = 0; i < m_d; i++) {
		double x_r = rand() / double(RAND_MAX) * (m_max[i] - m_min[i]) + m_min[i];
		double v_r = rand() / double(RAND_MAX) * (m_max[i] - m_min[i]) + m_min[i] - x_r;
		/*p1->m_Vec[i] = double(m_tMax + t) / (2.0 * double(m_tMax)) * v_r +
			double(m_tMax - t) / (2.0 * double(m_tMax)) * p2->m_Vec[i];*/
		p1->m_Vec[i] = double(m_tMax + t) / (2.0 * double(m_tMax)) * p2->m_Vec[i] +
			double(m_tMax - t) / (2.0 * double(m_tMax)) * v_r;
	}

	// ��������λ�á���������ֵ���ֲ�����λ�á��ֲ�����ֵ-ȡ���
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
