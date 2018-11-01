/***************************************************************
*  本文件包含粒子群算法类中关于GPU的函数的定义                        *
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
 * @brief 在CPU上运行
 */
void PSO::Run_GPU() {
	// 初始化cuda
	if (!psocudalib::InitCUDA()) {
		cout << "CUDA initializa failed!" << endl;
		return;
	}

	// 内存大小
	unsigned int memoriaSize1 = (m_d * m_number) * sizeof(double);
	unsigned int memoriaSize2 = (m_number) * sizeof(double);
	unsigned int memoriaSize3 = (m_d) * sizeof(double);

	double *xx;       // 位置
	double *vx;       // 速度
	double *value;    // 目标函数值
	double *pbestx;   // 个体最优位置
	double *pbest;    // 个体最优值
	double *min;      // 位置下界
	double *max;      // 位置上界
	int *gbest;       // 最优位置的标记?

	// 申请内存
	cudaMalloc((void**)&xx, memoriaSize1);
	cudaMalloc((void**)&vx, memoriaSize1);
	cudaMalloc((void**)&pbestx, memoriaSize1);
	cudaMalloc((void**)&pbest, memoriaSize2);
	cudaMalloc((void**)&max, memoriaSize3);
	cudaMalloc((void**)&min, memoriaSize3);

	cudaMalloc((void**)&value, memoriaSize2);//?
	cudaMalloc((void**)&gbest, sizeof(int));//?

	// 数据传递CPU->GPU
	cudaMemcpy(min, m_min, memoriaSize3, cudaMemcpyHostToDevice);
	cudaMemcpy(max, m_max, memoriaSize3, cudaMemcpyHostToDevice);

	dim3 threads(16, 16);
	dim3 blocks(2, 2);

	dim3 threadsN(6, 6);
	dim3 blocksN(1, 1);
	// 初始化粒子
	psokernel::InitParticles << <blocks, threads >> > (xx, vx, pbestx, gbest, m_d, m_number, min, max);

	int iter = 0;
	do {
		psokernel::GetFitness << <blocksN, threadsN >> > (xx, value, m_d, m_number, 1); //计算目标函数值
		psokernel::UpdatePbest << <blocks, threads >> > (xx, value, pbestx, pbest, m_d, m_number, iter); //更新个体最优
		psokernel::UpdateGbest << <blocksN, threadsN, 2 * m_number * sizeof(double) >> > (gbest, pbest, m_d, m_number, 16);//更新全局最优
		psokernel::UpdateParticles << <blocks, threads >> > (xx, vx, pbestx, gbest, m_d, m_number, min, max);// 更新粒子位置
		iter++;
	} while (iter < m_tMax); //m_tMax

	m_t_act = iter;

	// 取结果
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
	__global__ void InitParticles(double *xx, double *vx, double *pbestx, int *gbest, int d, int n, double *min, double *max) {
		int bx = blockIdx.x;
		int by = blockIdx.y;
		int tx = threadIdx.x;
		int ty = threadIdx.y;
		// 计算编号
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
	 * @breif 求目标函数值
	 * @param xx      位置
	 * @param value   目标函数值
	 * @param d       维数
	 * @param n       粒子个数
	 * @param funcIdx 目标函数序号
	 */
	__global__ void GetFitness(double *xx, double *value, int d, int n, int funcIdx) {
		int tx = threadIdx.x;
		int ty = threadIdx.y;
		int index = ty * blockDim.x + tx; // 粒子序号
		int offset = index * d; // 粒子在xx数组中的偏移（位置）

		if (index < n) {
			double val = 0.0;
			val = psocudalib::sphere(xx, d, offset);

			value[index] = val;
		}
	}

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
	__global__ void UpdatePbest(double *xx, double *value, double *pbestx, double *pbest,int d, int n, int iter) {
		int bx = blockIdx.x;
		int by = blockIdx.y;
		int tx = threadIdx.x;
		int ty = threadIdx.y;
		// 计算编号 有共享内存的问题等待优化
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
	 * @breif 更新个体最优
	 * @param gbest      全局最优标记
	 * @param pbest      个体最优值
	 * @param d          维数
	 * @param n          粒子个数
	 * @param threadNum  线程数 大于等于粒子个数的最小2的幂
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
			// 数据复制到共享内存
			indiceshared[index] = index;
			pbestshared[index] = pbest[index];
			//printf("%d, %f, %f\n", index, pbestshared[index], pbest[index]);

			__syncthreads();

			// 比较树
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

		// 取出全局最优的标记
		if (index == 0) {
			*gbest = indiceshared[0];
		}
	}

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
	__global__ void UpdateParticles(double *xx, double *vx, double *pbestx, int *gbest, int d, int n, double *min, double *max) {
		int bx = blockIdx.x;
		int by = blockIdx.y;
		int tx = threadIdx.x;
		int ty = threadIdx.y;
		// 计算编号 有共享内存的问题等待优化
		int index = bx * gridDim.x * blockDim.x * blockDim.y + by * blockDim.x * blockDim.y + ty * blockDim.x + tx;

		if (index < d * n) {
			int d_curr = index % d;
			double chi = 0.7298437881283576;
			double c = 2.05;
			double r1 = psocudalib::randomNumber(index);
			double r2 = psocudalib::randomNumber(index);
			vx[index] = chi * (vx[index] + c * r1 * (pbestx[index] - xx[index]) + c * r2 * (pbestx[d_curr + *gbest * d] - xx[index]));

			// 目前没有限制速度大小
			xx[index] = xx[index] + vx[index];

			// 范围 xx[index] = (max[dind] - min[dind]) * psocudalib::randomNumber(index) + min[dind];
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
