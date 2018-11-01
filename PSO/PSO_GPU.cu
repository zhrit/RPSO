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
		psokernel::UpdatePbest << <blocks, threads >> > (xx, value, pbestx, pbest, m_d, m_number, iter); //���¸�������
		psokernel::UpdateGbest << <blocksN, threadsN, 2 * m_number * sizeof(double) >> > (gbest, pbest, m_d, m_number, 16);//����ȫ������
		psokernel::UpdateParticles << <blocks, threads >> > (xx, vx, pbestx, gbest, m_d, m_number, min, max);// ��������λ��
		iter++;
	} while (iter < m_tMax); //m_tMax

	m_t_act = iter;

	// ȡ���
	//double *rxx = new double[m_d * m_number];
	//double *rvx = new double[m_d * m_number];
	//double *rv = new double[m_number];
	double *rpx= new double[m_d * m_number];
	double *rp = new double[m_number];
	int *rg = new int;

	//cudaMemcpy(rxx, xx, memoriaSize1, cudaMemcpyDeviceToHost);
	//cudaMemcpy(rvx, vx, memoriaSize1, cudaMemcpyDeviceToHost);
	//cudaMemcpy(rv, value, memoriaSize2, cudaMemcpyDeviceToHost);
	cudaMemcpy(rpx, pbestx, memoriaSize1, cudaMemcpyDeviceToHost);
	cudaMemcpy(rp, pbest, memoriaSize2, cudaMemcpyDeviceToHost);
	cudaMemcpy(rg, gbest, sizeof(int), cudaMemcpyDeviceToHost);

	for (int i = 0; i < m_d; i++) {
		m_result_pos[i] = rpx[m_d * *rg + i];
	}
	m_result_value = rp[*rg];

	cudaFree(xx);
	cudaFree(vx);
	cudaFree(pbestx);
	cudaFree(pbest);
	cudaFree(value);
	cudaFree(gbest);
	cudaFree(min);
	cudaFree(max);
	cudaThreadExit();
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
	__global__ void UpdatePbest(double *xx, double *value, double *pbestx, double *pbest,int d, int n, int iter) {
		int bx = blockIdx.x;
		int by = blockIdx.y;
		int tx = threadIdx.x;
		int ty = threadIdx.y;
		// ������ �й����ڴ������ȴ��Ż�
		int index = bx * gridDim.x * blockDim.x * blockDim.y + by * blockDim.x * blockDim.y + ty * blockDim.x + tx;

		int n_curr = index / d;

		if (index < d * n) {
			if (iter == 0) {
				pbestx[index] = xx[index];
				pbest[n_curr] = value[n_curr];
			}
			else {
				if (value[n_curr] < pbest[n_curr]) {
					pbestx[index] = xx[index];
					pbest[n_curr] = value[n_curr];
				}
			}
		}
	}


	/**
	 * @breif ���¸�������
	 * @param gbest      ȫ�����ű��
	 * @param pbest      ��������ֵ
	 * @param d          ά��
	 * @param n          ���Ӹ���
	 * @param threadNum  �߳��� ���ڵ������Ӹ�������С2����
	 */
	__global__ void UpdateGbest(int *gbest, double *pbest, int d, int n, int threadNum) {
		extern __shared__ double s[];
		double *pbestshared = s;
		int *indiceshared = (int*)&pbestshared[n];

		int tx = threadIdx.x;
		int ty = threadIdx.y;
		int index = tx + ty * blockDim.x;

		//printf("%d, %d\n", indiceshared, pbestshared);

		if (index < n) {
			// ���ݸ��Ƶ������ڴ�
			indiceshared[index] = index;
			pbestshared[index] = pbest[index];
			//printf("%d, %f, %f\n", index, pbestshared[index], pbest[index]);

			__syncthreads();

			// �Ƚ���
			for (int s = 1; s <= threadNum; s *= 2) {
				if (index % (2 * s) == 0) {
					if (index + s < n) {
						if (pbestshared[indiceshared[index]] > pbestshared[indiceshared[index + s]]) {
							indiceshared[index] = indiceshared[index + s];
						}
					}
				}
				__syncthreads();
			}
		}

		// ȡ��ȫ�����ŵı��
		if (index == 0) {
			*gbest = indiceshared[0];
		}
	}

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
	__global__ void UpdateParticles(double *xx, double *vx, double *pbestx, int *gbest, int d, int n, double *min, double *max) {
		int bx = blockIdx.x;
		int by = blockIdx.y;
		int tx = threadIdx.x;
		int ty = threadIdx.y;
		// ������ �й����ڴ������ȴ��Ż�
		int index = bx * gridDim.x * blockDim.x * blockDim.y + by * blockDim.x * blockDim.y + ty * blockDim.x + tx;

		if (index < d * n) {
			int d_curr = index % d;
			double chi = 0.7298437881283576;
			double c = 2.05;
			double r1 = psocudalib::randomNumber(index);
			double r2 = psocudalib::randomNumber(index);
			vx[index] = chi * (vx[index] + c * r1 * (pbestx[index] - xx[index]) + c * r2 * (pbestx[d_curr + *gbest * d] - xx[index]));

			// Ŀǰû�������ٶȴ�С
			xx[index] = xx[index] + vx[index];

			// ��Χ xx[index] = (max[dind] - min[dind]) * psocudalib::randomNumber(index) + min[dind];
			if (xx[index] > max[d_curr]) {
				xx[index] = max[d_curr];
				vx[index] = -vx[index] * 0.5;
			}
			else if (xx[index] < min[d_curr]) {
				xx[index] = min[d_curr];
				vx[index] = -vx[index] * 0.5;
			}
		}
	}
}
