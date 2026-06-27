#pragma once

#include "SimOneServiceAPI.h"
#include "SimOneHDMapAPI.h"
#include "SimOneSensorAPI.h"
#include "SSD/SimPoint3D.h"
#include "SSD/SimString.h"

#include "SampleGetNearMostLane.h"
#include "SampleGetNearLanes.h"
#include "SampleGetNearLanesWithAngle.h"
#include "SampleGetDistanceToLaneBoundary.h"
#include "SampleGetLaneSample.h"
#include "SampleGetLaneLink.h"
#include "SampleGetLaneType.h"
#include "SampleGetLaneWidth.h"
#include "SampleGetLaneST.h"
#include "SampleGetRoadST.h"
#include "SampleContainsLane.h"
#include "SampleGenerateRoute.h"
#include "SampleGetCrossHatchList.h"
#include "SampleGetTrafficLightList.h"
#include "SampleGetCrosswalkList.h"
#include "SampleGetHeights.h"
#include "SampleGetInertialFromLaneST.h"
#include "SampleGetLaneMiddlePoint.h"
#include "SampleGetParkingSpaceList.h"
#include "SampleGetRoadMark.h"
#include "SampleGetStoplineList.h"
#include "SampleGetTrafficSignList.h"
#include "SampleNavigate.h"
#include "SampleGetLaneSampleByLocation.h"
#include "SampleIsInsideLane.h"
#include "SampleIsInJunction.h"
#include "SampleIsDriving.h"
#include "SampleGetLaneLength.h"
#include "SampleGetSectionLaneList.h"
#include "SampleIsTwoSideRoad.h"
#include "SampleGetRoadLength.h"
#include "SampleGetJunctionList.h"
#include "SampleGetLaneData.h"

#include <chrono>
#include <thread>
#define M_PI        3.14159265358979323846

void SamplesHDMapByLocation(const SSD::SimPoint3D& pos)
{
	std::cout.precision(8);
	//1. GetNearMostLane
	SSD::SimString laneId = SampleGetNearMostLane(pos);
	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "laneId: %s", laneId.GetString());
	//2. GetNearLanes
	SampleGetNearLanes(pos, 5);
	//3. GetNearLanesWithAngle
	double headingAngle = M_PI / 180 * 30;
	double shiftAngle = M_PI / 180 * 90;
	SampleGetNearLanesWithAngle(pos, 5, headingAngle, shiftAngle);
	//4. GetDistanceToLaneBoundary
	SampleGetDistanceToLaneBoundary(pos);
	//5. GetLaneSample
	SSD::SimString laneName = SampleGetLaneSample(laneId);
	//6. GetLaneLink
	SampleGetLaneLink(laneId);
	//7. GetLaneType
	SampleGetLaneType(laneId);
	//8. GetLaneWidth
	SampleGetLaneWidth(laneId, pos);
	//9. GetLaneST
	double s = 0;
	double t = 0;
	SampleGetLaneST(laneId, pos, s, t);
	//10. GetRoadST
	SampleGetRoadST(laneId, pos);
	//11. GetInertialFromLaneST
	SampleGetInertialFromLaneST(laneId, s, t);
	//12. ContainsLane
	SampleContainsLane(laneId);
	//13. GetParkingSpaceList
	SSD::SimVector<HDMapStandalone::MParkingSpace> parkSpace = SampleGetParkingSpaceList();
	std::cout << "parkSpace =" << parkSpace.size() << std::endl;
	//14. GenerateRoute
	SampleGenerateRoute();
	//15. Navigate
	SSD::SimPoint3DVector inputPoints;
	inputPoints.push_back(SSD::SimPoint3D(-131.75125, 40.961385, 0));
	inputPoints.push_back(SSD::SimPoint3D(38.001163, 37.796493, 0));
	SSD::SimVector<long> roadList = SampleNavigate(inputPoints);
	//16. GetRoadMark
	SampleGetRoadMark(laneId, pos);
	//17. GetTrafficLightList
	SSD::SimVector<HDMapStandalone::MSignal> trafficLight = SampleGetTrafficLightList();
	//18. GetTrafficSignList
	SampleGetTrafficSignList();
	//19. GetStoplineList
	SampleGetStoplineList(laneId, trafficLight);
	//20. GetCrosswalkList
	SampleGetCrosswalkList(pos, laneId, trafficLight);
	//21. GetCrossHatchList
	SampleGetCrossHatchList(laneId);
	//22. GetLaneMiddlePoint
	SampleGetLaneMiddlePoint(pos, laneId);
	//23. GetHeights
	SampleGetHeights(pos, 10.0);
	//24. GetLaneData
	SSD::SimVector<HDMapStandalone::MLaneInfo> data;
	SampleGetLaneData(data);
	//25. GetJunctionList
	SampleGetJunctionList(laneId);
	//28. GetLaneLength
	SampleGetLaneLength(laneId);
	//29. GetSectionLaneList
	SSD::SimStringVector sectionLaneList;
	SampleGetSectionLaneList(laneId, sectionLaneList);
	//30. IsDriving
	SampleIsDriving(laneId);
	//32. IsInsideLane
	SSD::SimPoint3D inputPt(-64.251235, 48.126736, 0);
	HDMapStandalone::MSideState sideState;
	SampleIsInsideLane(inputPt, laneName, sideState);
	//33. GetLaneSampleByLocation
	HDMapStandalone::MLaneInfo info;
	SampleGetLaneSampleByLocation(inputPt, info);
}

void gpsCB(const char * mainVehicleID, SimOne_Data_Gps *gps)
{
	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "gpsCB: V: %d, GPS: %lld, pos:(%f, %f, %f)", mainVehicleID,
		gps->timestamp, gps->posX, gps->posY, gps->posZ);
	//HDMap samples based on gps location
	SSD::SimPoint3D pos(gps->posX, gps->posY, gps->posZ);
	SamplesHDMapByLocation(pos);
}

void SamplesGenerateRoute()
{
	SampleGenerateRoute();
}

int main(int argc, char* argv[])
{
	bool isJoinTimeLoop = false;
	const char* MainVehicleId = "0";
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop, "127.0.0.1");
	//SimOneAPI::StartSimOneNode(0,0,0);
	//while (true)
	//{
	//	if (SimOneAPI::SubMainVehicle(0))
	//		break;
	//}
	int timeout = 20;
	if (!SimOneAPI::LoadHDMap(timeout))
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "Failed to load hdmap!");
		return 0;
	}

	// Usage1. Samples of using HDMap by specifying location.
	/*SSD::SimPoint3D pos(0.0, 0.0, 0.0);
	SamplesHDMapByLocation(pos);*/

	// Usage2. Get position (of Ego e.g.) from sensers.
	std::unique_ptr<SimOne_Data_Gps> pGPS = std::make_unique<SimOne_Data_Gps>();
	SimOneAPI::SetGpsUpdateCB(gpsCB);
	while (1)
	{
		if (!SimOneAPI::GetGps(0/*vehicleId*/, pGPS.get()))
		{
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "Fetch GPS failed!");
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	return 0;
}

