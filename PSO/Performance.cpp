#include "pch.h"
#include "PSO.h"
#include "Benchmark.h"
#include "Performance.h"
#include <Windows.h>

int Benchmark::T = 0;

Performance::Performance() {
	//FuncPool = {Benchmark::Sphere};
}
/**
 * @breif 给定维度，给定粒子总数，测试性能
 * @ param serial 目标函数序号
 * @ param d      维数
 * @ param min    自变量下界
 * @ param min    自变量上界
 * @ param n      粒子个数
 * @ param repeat 重复次数
 */
PerfInfo Performance::SingleTest(int serial, int d, double *min, double *max, int n, int repeat, int tmax) {

	SetRandomSeed();

	PSO *pso = new PSO();
	double opt = FuncOptimals[serial];
	if (serial == 7) {
		opt *= d;
	}
	pso->Initialize(FuncPool[serial], d, min, max, n, tmax, opt, FuncPrec[serial]);

	PerfInfo info;
	info.PopulationSize = n;
	info.dem = d;
	info.Repeat = repeat;

	info.Time = 0;
	info.Iterations = 0;
	info.Time_S = 0;
	info.Time_O = 0;
	info.Iterations_S = 0;
	int successTime = 0;
	double beginTime = GetTickCount();
	for (int i = 0; i < repeat; i++) {
		//cout << i << endl;
		double beginTime_S = GetTickCount();
		pso->SetOption("useResample", 1);
		pso->SetOption("resampleMethod", 0);
		pso->Run();
		double costTime_S = GetTickCount() - beginTime_S;
		info.Iterations += pso->GetTAct();
		if (pso->GetStatus() == SUCCESS) {
			successTime++;
			info.Iterations_S += pso->GetTAct();
			info.Time_S += costTime_S;
			info.Time_O += pso->GetTObj();
		}
	}
	info.Time = GetTickCount() - beginTime;
	info.Iterations /= repeat;
	info.SuccessRate = double(successTime) / double(repeat);
	info.Time /= repeat;
	if (successTime > 0) {
		info.Iterations_S /= successTime;
		info.Time_S /= successTime;
		info.Time_O /= successTime;
	}
	delete pso;
	pso = nullptr;
	return info;
}

/**
 * @breif 给定维度，粒子总数对性能的影响
 * @ param serial   目标函数序号
 * @ param d        维数
 * @ param min      自变量下界
 * @ param min      自变量上界
 * @ param repeat   重复次数
 * @ param fileName 保存数据的文件名
 * @ param n_l      粒子总数范围下界
 * @ param n_u      粒子总数范围上界
 */
void Performance::EffectOfPopulation(int serial, int d, double *min, double *max, int repeat, int tmax, string fileName, int n_l, int n_u) {
	using namespace std;

	/*写文件中表格标题*/
	// 文件名字，如果传入了文件名字，则使用传入的名字，如果传入空字符串，则生成名字
	// performance_维数_年月日时分秒
	if (fileName == "") {
		//当前时间
		struct tm t_tm;          //tm结构指针
		time_t now;              //声明time_t类型变量
		time(&now);              //获取系统日期和时间
		localtime_s(&t_tm, &now);//获取当地日期和时间
		char ch[64] = { 0 };
		strftime(ch, sizeof(ch) - 1, "%Y%m%d_%H%M%S", &t_tm);
		//当前时间
		fileName = "E:\\postgraduate\\【毕设】RPSO\\data\\performance_f" + to_string(serial + 1) +
			"_d" + to_string(d) + "_" + ch + ".csv";
	}
	// 定义文件输出流 
	ofstream oFile;
	// 打开要输出的文件 
	oFile.open(fileName, ios::out | ios::app);

	oFile << "函数" << serial + 1 << "，维数" << d << "，重复次数" << repeat << endl;

	oFile.close();
	/*写文件中表格标题*/

	vector<PerfInfo> infos;
	for (int n = 3; n <= 40; n++) {
		PerfInfo info;
		if (n < n_l || n> n_u) {
			info.PopulationSize = n;
			info.dem = d;
			info.Repeat = repeat;
			info.Iterations = info.Iterations_S = 0;
			info.SuccessRate = info.Time = info.Time_S = info.Time_O = 0.0;
		}
		else {
			cout << "d = " << d << ", n = " << n << endl;
			info = SingleTest(serial, d, min, max, n, repeat, tmax);
		}
		infos.push_back(info);
	}
	WriteToCSV(infos, fileName);
}

/**
 * @breif 不同维度维度，粒子总数对性能的影响
 * @ param serial   目标函数序号
 * @ param min      自变量下界
 * @ param min      自变量上界
 * @ param repeat   重复次数
 * @ param fileName 保存数据的文件名
 */
void Performance::EffectOfPopulation_D(int serial, double *min, double *max, int repeat, int tmax, string fileName) {
	using namespace std;
	// 处理min、max
	for (int i = 0; i < 20; i++) {
		min[i] *= FuncSpaceSize[serial];
		max[i] *= FuncSpaceSize[serial];
	}
	/*写文件中表格标题*/
	// 文件名字，如果传入了文件名字，则使用传入的名字，如果传入空字符串，则生成名字
	// performance_维数_年月日时分秒
	if (fileName == "") {
		//当前时间
		struct tm t_tm;          //tm结构指针
		time_t now;              //声明time_t类型变量
		time(&now);              //获取系统日期和时间
		localtime_s(&t_tm, &now);//获取当地日期和时间
		char ch[64] = { 0 };
		strftime(ch, sizeof(ch) - 1, "%Y%m%d_%H%M%S", &t_tm);
		//当前时间
		fileName = "E:\\postgraduate\\【毕设】RPSO\\data\\performance_f" + to_string(serial + 1) +
			"_d2-30" + "_" + ch + ".csv";
	}
	//EffectOfPopulation(serial, 2, min, max, repeat, tmax, fileName, 3, 20);
	EffectOfPopulation(serial, 2, min, max, repeat, tmax, fileName, 5, 5);
	//EffectOfPopulation(serial, 10, min, max, repeat, tmax, fileName, 3, 20);
	//EffectOfPopulation(serial, 15, min, max, repeat, tmax, fileName, 3, 20);
	//EffectOfPopulation(serial, 20, min, max, repeat, tmax, fileName, 3, 20);
}

/**
 * @brief 把结果写入excel文件
 * @param infos 运行结果
 */
void Performance::WriteToCSV(const vector<PerfInfo> &infos, const string fileName) {
	using namespace std;
	// 定义文件输出流 
	ofstream oFile;

	// 打开要输出的文件 
	oFile.open(fileName, ios::out | ios::app); //trunc
	//oFile << endl;
	// 写入粒子个数N
	oFile << "N" << ",";
	vector<PerfInfo>::const_iterator iter;
	for (iter = infos.begin(); iter != infos.end(); iter++) {
		oFile << iter->PopulationSize << ",";
	}
	oFile << endl;

	// 写入成功率r
	oFile << "成功率(r)" << ",";
	for (iter = infos.begin(); iter != infos.end(); iter++) {
		oFile << iter->SuccessRate << ",";
	}
	oFile << endl;

	// 写入成功试验平均迭代次数T_act_s
	oFile << "成功试验迭代次数(T_s)" << ",";
	for (iter = infos.begin(); iter != infos.end(); iter++) {
		oFile << iter->Iterations_S << ",";
	}
	oFile << endl;

	// 写入平均迭代次数T_act
	oFile << "迭代次数(T)" << ",";
	for (iter = infos.begin(); iter != infos.end(); iter++) {
		oFile << iter->Iterations << ",";
	}
	oFile << endl;

	// 写入成功试验平均目标函数执行次数t_o
	oFile << "目标函数执行次数(t_o)" << ",";
	for (iter = infos.begin(); iter != infos.end(); iter++) {
		oFile << iter->Time_O << ",";
	}
	oFile << endl;

	// 写入成功试验平均运行时间t_s
	oFile << "成功试验耗时(t_s/ms)" << ",";
	for (iter = infos.begin(); iter != infos.end(); iter++) {
		oFile << iter->Time_S << ",";
	}
	oFile << endl;

	// 写入平均运行时间t
	oFile << "耗时(t/ms)" << ",";
	for (iter = infos.begin(); iter != infos.end(); iter++) {
		oFile << iter->Time << ",";
	}
	oFile << endl;

	oFile.close();
}

/**
 * 设置随机树种子
 */
void Performance::SetRandomSeed() {
	if (HasSetRandomSeed) {
		// 如果已经设置过，则返回，保证只设置一次
		return;
	}
	else {
		srand((unsigned int)time(0));
		HasSetRandomSeed = true;
	}
}
