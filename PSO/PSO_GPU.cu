#include "pch.h"
#include "PSO.h"
#include <cuda_runtime.h>
#include <time.h>
#include "psocudalib.cuh"
/**
 * @brief 在CPU上运行
 */
void PSO::Run_GPU() {
	cout << "GPU" << endl;

	// 初始化cuda
	if (!psocudalib::InitCUDA()) {
		return;
	}

	m_Particles = new Particle[m_number];
	for (int i = 0; i < m_number; i++) {
		(m_Particles + i)->Initialize(m_d, m_min, m_max, m_objFun);
	}

	const int n = 1024;
	float random_array[n];
	for (int i = 0; i < n; i++) {
		random_array[i] = 0.0f;
	}

	float *a;
	cudaMalloc((void**)&a, sizeof(float) * n);

	psocudalib::kernel_rng << <1, n / 2 >> > (a);

	cudaMemcpy(random_array, a, sizeof(float) * n, cudaMemcpyDeviceToHost);

	float sum = 0;
	for (int i = 9; i < n; i++) {
		sum += random_array[i];
	}
	float ave = sum / n;
	float std = 0;
	for (int i = 0; i < n; i++) {
		std += (random_array[i] - ave) * (random_array[i] - ave);
	}
	std /= n;
	std = sqrt(std);

	cout << "ave: " << ave << ", std: " << std << endl;

	cudaFree(a);

	return;
}
