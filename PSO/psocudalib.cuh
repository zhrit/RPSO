#ifndef _PSOCUDALIB_H_
#define _PSOCUDALIB_H_

#include <cuda_runtime.h>
#include <iostream>

using namespace std;
/**
 * cuda相关函数
 */
namespace psocudalib {
	__global__ void kernel_rng(float *data);
	__device__ double randomNumber(int id);

	__device__ double sphere(double *xx, int d, int offset);

	void printDeviceProp(const cudaDeviceProp &prop);
	/**
	 * @brief CUDA初始化函数
	 */
	bool InitCUDA() {
		int count;

		cudaGetDeviceCount(&count);
		if (count == 0) {
			cout << "There is no device" << endl;
			return false;
		}

		int i;
		for (i = 0; i < count; i++) {
			cudaDeviceProp prop;
			if (cudaGetDeviceProperties(&prop, i) == cudaSuccess) {
				if (prop.major >= 1) {
					//printDeviceProp(prop);
					break;
				}
			}
		}

		if (i == count) {
			cout << "There is no device supporting CUDA" << endl;
			return false;
		}

		cudaSetDevice(i);

		return true;
	}

	/**
	 * @brief 输出显卡配置
	 */
	void printDeviceProp(const cudaDeviceProp &prop) {
		printf("Device Name : %s.\n", prop.name);
		printf("totalGlobalMem : %zd.\n", prop.totalGlobalMem);
		printf("sharedMemPerBlock : %zd.\n", prop.sharedMemPerBlock);
		printf("regsPerBlock : %d.\n", prop.regsPerBlock);
		printf("warpSize : %d.\n", prop.warpSize);
		printf("memPitch : %zd.\n", prop.memPitch);
		printf("maxThreadsPerBlock : %d.\n", prop.maxThreadsPerBlock);
		printf("maxThreadsDim[0 - 2] : %d %d %d.\n", prop.maxThreadsDim[0], prop.maxThreadsDim[1], prop.maxThreadsDim[2]);
		printf("maxGridSize[0 - 2] : %d %d %d.\n", prop.maxGridSize[0], prop.maxGridSize[1], prop.maxGridSize[2]);
		printf("totalConstMem : %zd.\n", prop.totalConstMem);
		printf("major.minor : %d.%d.\n", prop.major, prop.minor);
		printf("clockRate : %d.\n", prop.clockRate);
		printf("textureAlignment : %zd.\n", prop.textureAlignment);
		printf("deviceOverlap : %d.\n", prop.deviceOverlap);
		printf("multiProcessorCount : %d.\n", prop.multiProcessorCount);
	}

	/**
	 * 随机数生成测试函数
	 */
	__global__ void kernel_rng(float *data) {
		int x = threadIdx.x;
		float r1 = randomNumber(x);
		float r2 = randomNumber(x);
		data[x] = r1;
		data[x + 512] = r2;
		printf("id: %d, r1: %6.2f, r2: %6.2f\n", x, r1, r2);
	}

	/**
	 * 随机数生成函数
	 */
	__device__ double randomNumber(int id) {
		int n;
		n = clock() + clock() * id;
		n = n ^ (n << 21);
		// n = n ^ (n >> 35);
		n = n ^ (n << 4);
		if (n < 0) n = -n;
		double n_f = double(n) / 2147483647.0f;//2141041196;//2147483647.0f;
		return n_f;
	}

	__device__ double sphere(double *xx, int d, int offset) {
		double result = 0.0;
		for (int i = 0; i < d; i++) {
			result += xx[offset + i] * xx[offset + i];
		}
		return result;
	}
}

#endif
