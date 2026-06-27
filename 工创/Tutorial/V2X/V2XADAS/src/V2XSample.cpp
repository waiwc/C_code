#include "SimOneSensorAPI.h"
#include "SimOneV2XAPI.h"
#include "SimOneServiceAPI.h"

#include <thread> 
#include <chrono>
#include <iostream>
#include <cstring>

int main(int argc, char* argv[])
{
	bool isJoinTimeLoop = false;
	const char* MainVehicleId = "0";
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);

	std::unique_ptr<SimOne_Data_V2XNFS> pDetections = std::make_unique<SimOne_Data_V2XNFS>();

	auto function = [](const char* mainVehicleId, const char* sensorId, SimOne_Data_V2XNFS *pDetections) {
		std::cout << pDetections->MsgFrameData << std::endl;
	};
	SimOneAPI::SetV2XInfoUpdateCB(function);

	while (1)
	{
		if (SimOneAPI::GetV2XInfo(MainVehicleId, "obu1", ESimOne_V2X_MessageFrame_PR_bsmFrame, pDetections.get())) {
			std::cout << "strlen = " << strlen(pDetections->MsgFrameData) << "  " << pDetections->MsgFrameData << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}
	}
	return 0;
}
