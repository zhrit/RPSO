/***************************************************************
*  ���ļ���������Ⱥ�㷨���й���GPU�ĺ����Ķ���                        *
*  @author   ZhRiT                                             *
*  @email    buaazhangrit@163.com                              *
*  @version  1.0.1                                             *
*  @date     2018-10-31                                        *
***************************************************************/
#include "pch.h"
#include "PSO.cuh"
#include <cuda_runtime.h>
#include <time.h>
#include "psocudalib.cuh"
#include <iostream>
#include <device_launch_parameters.h>

using namespace std;
/**
 * @brief ��CPU������
 */
void PSO::Run_GPU() {
	cout << "GPU" << endl;

	// ��ʼ��cuda
	if (!psocudalib::InitCUDA()) {
		cout << "CUDA initializa failed!" << endl;
		return;
	}

	// �ڴ��С
	unsigned int memoriaSize1 = (m_d * m_number) * sizeof(double);
	unsigned int memoriaSize2 = (m_number) * sizeof(double);
	unsigned int memoriaSize3 = (m_d) * sizeof(double);

	double *xx;       // λ��
	double *vx;       // �ٶ�
	double *value;    // Ŀ�꺯��ֵ
	double *pbestx;   // ��������λ��
	double *pbest;    // ��������ֵ
	double *min;      // λ���½�
	double *max;      // λ���Ͻ�
	int *gbest;       // ����λ�õı��?

	// �����ڴ�
	cudaMalloc((void**)&xx, memoriaSize1);
	cudaMalloc((void**)&vx, memoriaSize1);
	cudaMalloc((void**)&pbestx, memoriaSize1);
	cudaMalloc((void**)&pbest, memoriaSize2);
	cudaMalloc((void**)&max, memoriaSize3);
	cudaMalloc((void**)&min, memoriaSize3);

	cudaMalloc((void**)&value, memoriaSize2);//?
	cudaMalloc((void**)&gbest, sizeof(int));//?

	// ���ݴ���CPU->GPU
	cudaMemcpy(min, m_min, memoriaSize3, cudaMemcpyHostToDevice);
	cudaMemcpy(max, m_max, memoriaSize3, cudaMemcpyHostToDevice);

	dim3 threads(16, 16);
	dim3 blocks(2, 2);

	dim3 threadsN(6, 6);
	dim3 blocksN(1, 1);
	// ��ʼ������
	psokernel::InitParticles << <blocks, threads >> > (xx, vx, pbestx, gbest, m_d, m_number, min, max);

	int iter = 0;
	do {
		psokernel::GetFitness << <blocksN, threadsN >> > (xx, value, m_d, m_number, 1); //����Ŀ�꺯��ֵ
		iter++;
	} while (iter < 1); //m_tMax

	m_t_act = iter;

	double *rxx = new double[m_d * m_number];
	double *rvx = new double[m_d * m_number];
	double *rv = new double[m_number];

	cudaMemcpy(rxx, xx, memoriaSize1, cudaMemcpyDeviceToHost);
	cudaMemcpy(rvx, vx, memoriaSize1, cudaMemcpyDeviceToHost);
	cudaMemcpy(rv, value, memoriaSize2, cudaMemcpyDeviceToHost);

	cout << "x: ";
	for (int i = 0; i < m_d * m_number; i++) {
		if (i % m_d == 0) {
			cout << "\n";
		}
		cout << rxx[i] << ", ";
	}
	cout << "\nv: ";
	for (int i = 0; i < m_d * m_number; i++) {
		if (i % m_d == 0) {
			cout << "\n";
		}
		cout << rvx[i] << ", ";
	}
	cout << "\nvalue: ";
	for (int i = 0; i < m_number; i++) {
		cout << rv[i] << ", \n";
	}
	return;
}


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
	__global__ void InitParticles(double *xx, double *vx, double *pbestx, int *gbest, int d, int n, double *min, double *max) {
		int bx = blockIdx.x;
		int by = blockIdx.y;
		int tx = threadIdx.x;
		int ty = threadIdx.y;
		// ������
		int index = bx * gridDim.x * blockDim.x * blockDim.y + by * blockDim.x * blockDim.y + ty * blockDim.x + tx;

		if (index < d * n) {
			int dind = index % d;
			xx[index] = (max[dind] - min[dind]) * psocudalib::randomNumber(index) + min[dind];
			pbestx[index] = xx[index];

			vx[index] = (max[dind] - min[dind]) * psocudalib::randomNumber(index) + min[dind] - xx[index];

			if (index == 0) *gbest = 0;
		}
	}

	/**
	 * @breif ��Ŀ�꺯��ֵ
	 * @param xx      λ��
	 * @param value   Ŀ�꺯��ֵ
	 * @param d       ά��
	 * @param n       ���Ӹ���
	 * @param funcIdx Ŀ�꺯�����
	 */
	__global__ void GetFitness(double *xx, double *value, int d, int n, int funcIdx) {
		int tx = threadIdx.x;
		int ty = threadIdx.y;
		int index = ty * blockDim.x + tx; // �������
		int offset = index * d; // ������xx�����е�ƫ�ƣ�λ�ã�

		if (index < n) {
			double val = 0.0;
			val = psocudalib::sphere(xx, d, offset);

			value[index] = val;
		}
	}
}
