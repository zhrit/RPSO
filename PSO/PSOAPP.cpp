// PSO.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include "PSO.cuh"
#include "Benchmark.h"
#include "Performance.h"
#include <iostream>

int main()
{
	srand((unsigned int)time(0));
	double m1[20] = { -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, - 100, -100, -100, -100, -100, -100, -100, -100, -100, -100 };
	double m2[20] = { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 };
	PSO *pso = new PSO();
	pso->Initialize(Benchmark::Sphere, 10, m1, m2, 10, 1000, 0.0);
	pso->SetOption("model", 0);
	pso->SetOption("topology", 0);
	pso->SetOption("useResample", 0);
	pso->SetOption("resampleMethod", 4);
	pso->SetOption("parallel", 0);
	pso->Run();

	//double m1[20] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
	//double m2[20] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
	//Performance per;
	//per.EffectOfPopulation_D(1, m1, m2, 100, 1000, "E:\\postgraduate\\【毕设】RPSO\\data\\performance_f8_d2-30_20181022_155239.csv");
	//".\\data\\performance_f1_d2-30_20181015_225135.csv"

	/*cout << info.PopulationSize << ", " 
		<< info.Repeat << ", " 
		<< info.dem << ", " 
		<< info.SuccessRate << ", " 
		<< info.Iterations << ", " 
		<< info.Time << ", " 
		<< endl;*/
	//double a[4] = { -420.9687, -420.9687, -420.9687, -420.9687 };
	//cout << Benchmark::Generalized_Schwefel2_26(a, 2);
}


