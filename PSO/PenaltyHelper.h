#pragma once
#include <vector>

struct ResultInfo {
	double RealObj;
	std::vector<double> CTValue;
};

class PenaltyHelper
{
public:
	PenaltyHelper() {

	}
	~PenaltyHelper() {

	}

	static std::vector<ResultInfo> ResInfo;

	static void Init();
};

