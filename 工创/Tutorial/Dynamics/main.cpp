#include "SimOneServiceAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneSensorAPI.h"
#include "TestSample.hpp"
#include <string.h>

using namespace std;

#define EXIT_FAILURE 1

int main(int argc, char* argv[])
{
	if (SimOneAPI::InitSimOneAPI("0", false, "127.0.0.1"))
	{
		cout << "InitSimOneAPI Done" << endl;
	}

	SimOneAPI::SetDriverName("0", "DynaTest");

	std::string csvPath = "";
	for (int i = 1; i < argc; ++i) {
		if (strcmp(argv[i], "-csvPath") == 0) {
			csvPath = argv[i + 1];
			++i;
		}
		else {
			csvPath = "resultsALL.csv";
		}
	}

	TestSample test;

	/* 1. full brake */
	test.FullBrakeTest("0", 1.0f, csvPath);

	/* 2. accelerate to a certain speed */
	//test.DriveToSpeed(0, 100.f, 1.0f, csvPath);

	/* 3. dynamical steer */
	//test.SteerDynamicTest(0, 60.f, TestSample::SteerType::sineSteer, csvPath);
	return 0;
}