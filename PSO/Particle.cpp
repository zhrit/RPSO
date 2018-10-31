/***************************************************************
*  ���ļ�����������Ķ���                                          *
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

/*----------------------------------------------------------�����ඨ��--------------------------------------------------------------*/
Particle::Particle() {
	// ��������������ӣ���random()�������õ��������
	// srand((unsigned int)time(NULL));
}

Particle::~Particle() {
	// �ͷſռ䣬ָ���ÿ�
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
 * @brief ��ʼ������
 *   ������Ⱥ�㷨��ÿ�����ӵ��ã���ʼ��������Ϣ��
 *   �����������λ�á��ٶȡ���ʼ���������ź;ֲ����ŵȲ���
 * @param d      ά��
 * @param min    λ���½�
 * @param max    λ���Ͻ�
 * @param objFun Ŀ�꺯��
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
 * @brief ����������ӵ�λ�ú��ٶ�
 */
void Particle::Random() {
	for (int i = 0; i < m_Size; i++) {
		m_Pos[i] = rand() / double(RAND_MAX) * (m_max[i] - m_min[i]) + m_min[i];
		m_Vec[i] = rand() / double(RAND_MAX) * (m_max[i] - m_min[i]) + m_min[i] - m_Pos[i];
	}
	m_Value = CalValue();
}

/**
 * @brief ���������ź;ֲ����Ÿ���ֵ
 *   �����������ٶȺ�λ�ø������õ�
 *   �ֲ������ڻ������˺�����������õ�
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
 * @brief ���㵱ǰλ�������Ӧ��Ŀ�꺯��ֵ
 */
double Particle::CalValue() const {
	return m_objFun(m_Pos, m_Size);
}

/**
 * @brief ���¸������ź;ֲ�����
 * @param index min_max�Ż���־
 *   -0 min�Ż�
 *   -1 max�Ż�
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
 * @brief ������ǰ�ƶ�һ��
 * @param chi ѹ������
 * @param c1  ���ٶ�ϵ�������壩
 * @param c2  ���ٶ�ϵ����Ⱥ�壩
 * @param r1  ������ӣ����壩
 * @param r2  ������ӣ�Ⱥ�壩
 * @param gb  Ⱥ��Ը������Ӱ���λ��
 *   -nullptr ������˻�������ʱ����nullptr����ʱ�Զ�ʹ�þֲ�����λ��
 *   -gb      ȫ������ʱ����ȫ�����Ŵ���
 */
 // ÿ�����ӵ�ÿһά����ͬ������r1��r2
void Particle::Move(double chi, double c1, double c2, double r1, double r2, const double *gb) {
	if (gb == nullptr) gb = m_Pos_best_local;
	for (int i = 0; i < m_Size; i++) {
		m_Vec[i] = chi * (m_Vec[i] + c1 * r1 * (m_Pos_best[i] - m_Pos[i]) + c2 * r2 * (gb[i] - m_Pos[i]));
		m_Pos[i] += m_Vec[i];
		// ������Χ�Ĵ���
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
// ÿ����������ͬ������r1��r2
void Particle::Move(double chi, double c1, double c2, const double* r1, const double* r2, const double *gb) {
	if (gb == nullptr) gb = m_Pos_best_local;
	for (int i = 0; i < m_Size; i++) {
		m_Vec[i] = chi * (m_Vec[i] + c1 * r1[i] * (m_Pos_best[i] - m_Pos[i]) + c2 * r2[i] * (gb[i] - m_Pos[i]));
		m_Pos[i] += m_Vec[i];
		// ������Χ�Ĵ���
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
// ÿ�������ò�ͬ������r1��r2
void Particle::Move(double chi, double c1, double c2, const double *gb) {
	if (gb == nullptr) gb = m_Pos_best_local;
	for (int i = 0; i < m_Size; i++) {
		double r1 = rand() / (double)(RAND_MAX);
		double r2 = rand() / (double)(RAND_MAX);
		m_Vec[i] = chi * (m_Vec[i] + c1 * r1 * (m_Pos_best[i] - m_Pos[i]) + c2 * r2 * (gb[i] - m_Pos[i]));
		m_Pos[i] += m_Vec[i];
		// ������Χ�Ĵ���
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
 * @brief ����set����
 *   Ϊm_Pos_best_local��m_Value_best_local��ֵ
 * @param lb   �ֲ�����λ��
 * @param lbv  �ֲ�����ֵ
 */
void Particle::SetLocalBest(const double* lb, double lbv) {
	for (int i = 0; i < m_Size; i++) {
		m_Pos_best_local[i] = lb[i];
	}
	m_Value_best_local = lbv;
}

/**
 * @brief m_Value_best_local����get����
 * @return
 */
double Particle::GetLocalBestValue() const {
	return m_Value_best_local;
}

/**
 * @brief m_Value_best����get����
 * @return
 */
double Particle::GetSelfBestValue() const {
	return m_Value_best;
}

/**
 * @brief m_Pos_best����get����
 * @return ����ָ�룬�����޸�������
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