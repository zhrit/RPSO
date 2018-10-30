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
 * @breif ����ά�ȣ�����������������������
 * @ param serial Ŀ�꺯�����
 * @ param d      ά��
 * @ param min    �Ա����½�
 * @ param min    �Ա����Ͻ�
 * @ param n      ���Ӹ���
 * @ param repeat �ظ�����
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
 * @breif ����ά�ȣ��������������ܵ�Ӱ��
 * @ param serial   Ŀ�꺯�����
 * @ param d        ά��
 * @ param min      �Ա����½�
 * @ param min      �Ա����Ͻ�
 * @ param repeat   �ظ�����
 * @ param fileName �������ݵ��ļ���
 * @ param n_l      ����������Χ�½�
 * @ param n_u      ����������Χ�Ͻ�
 */
void Performance::EffectOfPopulation(int serial, int d, double *min, double *max, int repeat, int tmax, string fileName, int n_l, int n_u) {
	using namespace std;

	/*д�ļ��б�����*/
	// �ļ����֣�����������ļ����֣���ʹ�ô�������֣����������ַ���������������
	// performance_ά��_������ʱ����
	if (fileName == "") {
		//��ǰʱ��
		struct tm t_tm;          //tm�ṹָ��
		time_t now;              //����time_t���ͱ���
		time(&now);              //��ȡϵͳ���ں�ʱ��
		localtime_s(&t_tm, &now);//��ȡ�������ں�ʱ��
		char ch[64] = { 0 };
		strftime(ch, sizeof(ch) - 1, "%Y%m%d_%H%M%S", &t_tm);
		//��ǰʱ��
		fileName = "E:\\postgraduate\\�����衿RPSO\\data\\performance_f" + to_string(serial + 1) +
			"_d" + to_string(d) + "_" + ch + ".csv";
	}
	// �����ļ������ 
	ofstream oFile;
	// ��Ҫ������ļ� 
	oFile.open(fileName, ios::out | ios::app);

	oFile << "����" << serial + 1 << "��ά��" << d << "���ظ�����" << repeat << endl;

	oFile.close();
	/*д�ļ��б�����*/

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
 * @breif ��ͬά��ά�ȣ��������������ܵ�Ӱ��
 * @ param serial   Ŀ�꺯�����
 * @ param min      �Ա����½�
 * @ param min      �Ա����Ͻ�
 * @ param repeat   �ظ�����
 * @ param fileName �������ݵ��ļ���
 */
void Performance::EffectOfPopulation_D(int serial, double *min, double *max, int repeat, int tmax, string fileName) {
	using namespace std;
	// ����min��max
	for (int i = 0; i < 20; i++) {
		min[i] *= FuncSpaceSize[serial];
		max[i] *= FuncSpaceSize[serial];
	}
	/*д�ļ��б�����*/
	// �ļ����֣�����������ļ����֣���ʹ�ô�������֣����������ַ���������������
	// performance_ά��_������ʱ����
	if (fileName == "") {
		//��ǰʱ��
		struct tm t_tm;          //tm�ṹָ��
		time_t now;              //����time_t���ͱ���
		time(&now);              //��ȡϵͳ���ں�ʱ��
		localtime_s(&t_tm, &now);//��ȡ�������ں�ʱ��
		char ch[64] = { 0 };
		strftime(ch, sizeof(ch) - 1, "%Y%m%d_%H%M%S", &t_tm);
		//��ǰʱ��
		fileName = "E:\\postgraduate\\�����衿RPSO\\data\\performance_f" + to_string(serial + 1) +
			"_d2-30" + "_" + ch + ".csv";
	}
	//EffectOfPopulation(serial, 2, min, max, repeat, tmax, fileName, 3, 20);
	EffectOfPopulation(serial, 2, min, max, repeat, tmax, fileName, 5, 5);
	//EffectOfPopulation(serial, 10, min, max, repeat, tmax, fileName, 3, 20);
	//EffectOfPopulation(serial, 15, min, max, repeat, tmax, fileName, 3, 20);
	//EffectOfPopulation(serial, 20, min, max, repeat, tmax, fileName, 3, 20);
}

/**
 * @brief �ѽ��д��excel�ļ�
 * @param infos ���н��
 */
void Performance::WriteToCSV(const vector<PerfInfo> &infos, const string fileName) {
	using namespace std;
	// �����ļ������ 
	ofstream oFile;

	// ��Ҫ������ļ� 
	oFile.open(fileName, ios::out | ios::app); //trunc
	//oFile << endl;
	// д�����Ӹ���N
	oFile << "N" << ",";
	vector<PerfInfo>::const_iterator iter;
	for (iter = infos.begin(); iter != infos.end(); iter++) {
		oFile << iter->PopulationSize << ",";
	}
	oFile << endl;

	// д��ɹ���r
	oFile << "�ɹ���(r)" << ",";
	for (iter = infos.begin(); iter != infos.end(); iter++) {
		oFile << iter->SuccessRate << ",";
	}
	oFile << endl;

	// д��ɹ�����ƽ����������T_act_s
	oFile << "�ɹ������������(T_s)" << ",";
	for (iter = infos.begin(); iter != infos.end(); iter++) {
		oFile << iter->Iterations_S << ",";
	}
	oFile << endl;

	// д��ƽ����������T_act
	oFile << "��������(T)" << ",";
	for (iter = infos.begin(); iter != infos.end(); iter++) {
		oFile << iter->Iterations << ",";
	}
	oFile << endl;

	// д��ɹ�����ƽ��Ŀ�꺯��ִ�д���t_o
	oFile << "Ŀ�꺯��ִ�д���(t_o)" << ",";
	for (iter = infos.begin(); iter != infos.end(); iter++) {
		oFile << iter->Time_O << ",";
	}
	oFile << endl;

	// д��ɹ�����ƽ������ʱ��t_s
	oFile << "�ɹ������ʱ(t_s/ms)" << ",";
	for (iter = infos.begin(); iter != infos.end(); iter++) {
		oFile << iter->Time_S << ",";
	}
	oFile << endl;

	// д��ƽ������ʱ��t
	oFile << "��ʱ(t/ms)" << ",";
	for (iter = infos.begin(); iter != infos.end(); iter++) {
		oFile << iter->Time << ",";
	}
	oFile << endl;

	oFile.close();
}

/**
 * �������������
 */
void Performance::SetRandomSeed() {
	if (HasSetRandomSeed) {
		// ����Ѿ����ù����򷵻أ���ֻ֤����һ��
		return;
	}
	else {
		srand((unsigned int)time(0));
		HasSetRandomSeed = true;
	}
}
