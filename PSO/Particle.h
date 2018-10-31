/***************************************************************
*  ���ļ����������������                                          *
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
 * @brief ������
 * ��������Ⱥ�㷨��ÿ������Ӧ�þ߱������ݳ�Ա�ͷ���
 */
class Particle {
	friend ostream& operator<<(ostream &out, Particle &par);
public:
	Particle();
	virtual ~Particle();

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
	void Initialize(int d, double* min, double* max, double(*objFun)(double*, int));

	/**
	 * @brief ����������ӵ�λ�ú��ٶ�
	 */
	void Random();

	/**
	 * @brief ���������ź;ֲ����Ÿ���ֵ
	 *   �����������ٶȺ�λ�ø������õ�
	 *   �ֲ������ڻ������˺�����������õ�
	 */
	void InitBest();

	/**
	 * @brief ���㵱ǰλ�������Ӧ��Ŀ�꺯��ֵ
	 */
	double CalValue() const;

	/**
	 * @brief ���¸������ź;ֲ�����
	 * @param index min_max�Ż���־
	 *   -0 min�Ż�
	 *   -1 max�Ż�
	 */
	void UpdateBest(int index = 0);

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
	void Move(double, double, double, double, double, const double*);
	// ÿ����������ͬ������r1��r2
	void Move(double, double, double, const double*, const double*, const double*);
	// ÿ�������ò�ͬ������r1��r2
	void Move(double, double, double, const double*);

	/**
	 * @brief ����set����
	 *   Ϊm_Pos_best_local��m_Value_best_local��ֵ
	 * @param lb   �ֲ�����λ��
	 * @param lbv  �ֲ�����ֵ
	 */
	void SetLocalBest(const double*, double);

	/**
	 * @brief m_Value_best_local����get����
	 * @return
	 */
	double GetLocalBestValue() const;

	/**
	 * @brief m_Value_best����get����
	 * @return
	 */
	double GetSelfBestValue() const;

	/**
	 * @brief m_Pos_best����get����
	 * @return ����ָ�룬�����޸�������
	 */
	const double* GetSelfBestPos() const;

public:
	double *m_Pos;                              // ��ǰλ��
	double *m_Vec;                              // ��ǰ�ٶ�
	double m_Value;                             // ��ǰλ�ö�Ӧ��Ŀ�꺯��ֵ
	double *m_min;                              // λ���½�
	double *m_max;                              // λ���Ͻ�
	int m_Size;                                 // ά��
	bool isNull{ true };                        // �Ƿ�Ϊ�գ��Ƿ񾭹���ʼ����

	double *m_Pos_best;                         // ��������λ��
	double m_Value_best;                        // ��������ֵ

	double *m_Pos_best_local;                   // �ֲ�����λ��
	double m_Value_best_local;                  // �ֲ�����ֵ

	double(*m_objFun)(double*, int) { nullptr };// ����ָ��

	static int m_Tmax;                          // ����������
};