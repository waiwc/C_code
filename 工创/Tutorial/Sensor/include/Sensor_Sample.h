#ifndef SIMONESENSORAPI_SAMPLE_H
#define SIMONESENSORAPI_SAMPLE_H

#include <vector>
#include <thread>
#include <string>
// #include <fstream>
#include <math.h>

#include "Service/SimOneIOStruct.h"
#include "SimOneServiceAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneSensorAPI.h"
#include "SimOneV2XAPI.h"
#include "timer.hpp"

#include "dumper.h"

class sensorapi_sample
{
public:
	sensorapi_sample(const char* mv_id);
	~sensorapi_sample();

	int64_t getCurrentTime();

	void InitSimOneAPI(bool isJoinTimeLoop, const char *serverIP);

	// Sensor API
	void Test_GetEnvironment();
	void Test_GPS(bool IsCallBackMode);
	void Test_GetGroundTruth(bool IsCallBackMode);
	void Test_RadarDetection(bool IsCallBackMode);
	void Test_UltrasonicRadar();
	void Test_UltrasonicRadars(bool IsCallBackMode);
	void Test_GetSensorDetections(bool IsCallBackMode);
	void Test_GetSensorConfigurations();
	void Test_SetEnvironment();
	// GetTrafficLight: Test in Test_HDMap_ALL
	void Test_SensorLaneInfo(bool IsCallBackMode);

protected:
	std::string mainVehicleId;
	static dumper dbg_data;
};

#endif
