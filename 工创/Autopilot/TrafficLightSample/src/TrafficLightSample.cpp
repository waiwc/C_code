#include "SimOneSensorAPI.h"
#include "SimOneV2XAPI.h"
#include "SimOneServiceAPI.h"
#include "SimOneHDMapAPI.h"
#include "SimOnePNCAPI.h"
#include "public/common/MLaneInfo.h"
#include "public/common/MLaneId.h"
#include "UtilDriver.h"
#include "UtilMath.h"
#include <iostream>
#include "assert.h"

SimOne_Data_Gps Gps = SimOne_Data_Gps();
SimOne_Data_CaseInfo pCaseInfoTest = SimOne_Data_CaseInfo();
SimOne_Data_WayPoints WayPoints;

SSD::SimString GetNearMostLane(const SSD::SimPoint3D& pos)
{
	SSD::SimString laneId;
	double s, t, s_toCenterLine, t_toCenterLine;
	if (!SimOneAPI::GetNearMostLane(pos, laneId, s, t, s_toCenterLine, t_toCenterLine))
	{
		std::cout << "Error: lane is not found." << std::endl;
	}
	return std::move(laneId);
}

SSD::SimPoint3DVector GetLaneSample(const SSD::SimString& laneId)
{
	SSD::SimPoint3DVector targetPath;
	HDMapStandalone::MLaneInfo info;
	if (SimOneAPI::GetLaneSample(laneId, info))
	{
		for (auto& pt : info.centerLine)
		{
			targetPath.push_back(SSD::SimPoint3D(pt.x, pt.y, pt.z));
		}
	}
	return std::move(targetPath);
}

void GetLaneSampleFromS(const SSD::SimString& laneId, const double& s, SSD::SimPoint3DVector& targetPath)
{
	HDMapStandalone::MLaneInfo info;
	if (SimOneAPI::GetLaneSample(laneId, info))
	{
		double accumulated = 0.0;
		int startIndex = -1;
		for (unsigned int i = 0; i < info.centerLine.size() - 1; i++)
		{
			auto& pt = info.centerLine[i];
			auto& ptNext = info.centerLine[i + 1];
			double d = UtilMath::distance(pt, ptNext);
			accumulated += d;
			if (accumulated >= s)
			{
				startIndex = i + 1;
				break;
			}
		}
		for (unsigned int i = startIndex; i < info.centerLine.size(); i++)
		{
			SSD::SimPoint3D item = info.centerLine[i];
			targetPath.push_back(info.centerLine[i]);
		}
	}
}

HDMapStandalone::MSignal GetTargetLight(const HDMapStandalone::MLaneId& id, const SSD::SimString& laneId, const SSD::SimVector<long>& roadIdList)
{
	HDMapStandalone::MSignal light;
	SSD::SimVector<HDMapStandalone::MSignal> lightList;
	SimOneAPI::GetTrafficLightList(lightList);

	std::cout << "all light size:" << lightList.size() << std::endl;
	for (auto& item : lightList) {
		std::cout << "light id " << item.id << std::endl;
	}
	for (auto& item : lightList)
	{
		int num = 0;
		std::cout << "light id " << item.id << std::endl;
		std::cout << "validities size " << item.validities.size()<<std::endl;
		for (auto& ptValidities : item.validities)
		{
			std::cout << "old road id " << ptValidities.roadId << std::endl;
			auto it = std::find(roadIdList.begin(), roadIdList.end(), ptValidities.roadId);
			if (it != roadIdList.end())
			{
				std::cout << "alread find road id " << ptValidities.roadId << std::endl;
				++num;
			}
		}

		//current lane
		if (num >= 2)
		{
			light = item;
			std::cout << "light pos_x:" << item.pt.x << std::endl;
			std::cout << "light pos_y:" << item.pt.y << std::endl;
			std::cout << "valid road id------------ " << item.id << std::endl;
			break;
		}
	}
	return std::move(light);
}

bool IsGreenLight(const long& lightId, const SSD::SimString& laneId, const HDMapStandalone::MSignal& light)
{
	SimOne_Data_TrafficLight trafficLight;
	if (SimOneAPI::GetTrafficLight(0, lightId, &trafficLight))
	{
		std::cout << "trafficLight.status:----------------------" << trafficLight.status << std::endl;
		if (trafficLight.status != ESimOne_TrafficLight_Status::ESimOne_TrafficLight_Status_Green)
		{

			return false;
		}

		else 
		{
			std::cout << "light status: " << trafficLight.status << std::endl;
			std::cout << "trafficLight.countDown: " << trafficLight.countDown << std::endl;
			return int(trafficLight.countDown) > 10;
		}
	}
	else
	{
		return true;
	}
}

SSD::SimString GetTargetSuccessorLane(const SSD::SimStringVector& successorLaneNameList, const SSD::SimVector<HDMapStandalone::MSignalValidity>& validities)
{
	SSD::SimString targetSuccessorLaneId;
	for (auto &successorLane : successorLaneNameList) {
		HDMapStandalone::MLaneId id(successorLane);
		for (auto & validitie : validities) {
			if ((id.roadId == validitie.roadId && id.sectionIndex == validitie.sectionIndex && id.sectionIndex == validitie.fromLaneId)
				|| (id.roadId == validitie.roadId && id.sectionIndex == validitie.sectionIndex && id.sectionIndex == validitie.toLaneId))
			{
				targetSuccessorLaneId = successorLane;
				break;
			}
		}
	}
	return std::move(targetSuccessorLaneId);
}

SSD::SimPoint3D GetTragetStopLine(const HDMapStandalone::MSignal& light, const SSD::SimString& laneId)
{
	SSD::SimVector<HDMapStandalone::MObject> stoplineList;
	SimOneAPI::GetStoplineList(light, laneId, stoplineList);
	return stoplineList[0].pt;
}

void GetSuccessorTargetPath(const SSD::SimPoint3D& vehiclePos, const SSD::SimString& laneId, const SSD::SimString& targetSuccessorLaneId,
	const SSD::SimPoint3D& stopLine, SSD::SimPoint3DVector& targetPath)
{
	targetPath.clear();

	double s, t;
	bool found = SimOneAPI::GetLaneST(laneId, vehiclePos, s, t);
	if (!found)  //it not inside the lane that laneId specifies any more
	{
		return;
	}
	GetLaneSampleFromS(laneId, s, targetPath);

	//Add successor lane's knots
	HDMapStandalone::MLaneInfo info;
	if (SimOneAPI::GetLaneSample(targetSuccessorLaneId, info))
	{
		for (auto& pt : info.centerLine)
		{
			targetPath.push_back(SSD::SimPoint3D(pt.x, pt.y, pt.z));
		}
	}
}

bool IsInSafeDistance(const SSD::SimPoint3D& vehiclePos, const SSD::SimPoint3D& stopLine)
{
	SSD::SimPoint2D vehiclePos2D(vehiclePos.x, vehiclePos.y);
	double safeDistance = UtilMath::distance(vehiclePos2D, SSD::SimPoint2D(stopLine.x, stopLine.y));
	std::cout << "safeDistance " << safeDistance << std::endl;
	return int(safeDistance) > 15;
}


//Main function
//
int main()
{
	const char* MainVehicleId = "0";
	bool isJoinTimeLoop = false;
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);
	while (1) {
		int frame = SimOneAPI::Wait();
		SimOneAPI::GetGps(MainVehicleId,&Gps);
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Running && (Gps.timestamp > 0)) {
			printf("SimOne Initialized\n");
			SimOneAPI::NextFrame(frame);
			break;
		}
		printf("SimOne Initializing...\n");

	}

	int timeout = 20;
	bool slowDown = false;
	int obstacltFlag = 0;
	if (!SimOneAPI::LoadHDMap(timeout)) {
		std::cout << "Failed to load map!" << std::endl;
		return 0;
	}


	SimOneAPI::GetGps(0, &Gps);
	SSD::SimPoint3D MainVehiclePos(Gps.posX, Gps.posY, Gps.posZ);
	SSD::SimString laneId = GetNearMostLane(MainVehiclePos);
	SSD::SimPoint3DVector targetPath = GetLaneSample(laneId);

	HDMapStandalone::MLaneLink laneLink;
	SimOneAPI::GetLaneLink(laneId, laneLink); 
	auto& successorLaneNameList = laneLink.successorLaneNameList;

	HDMapStandalone::MLaneId id(laneId);
	std::cout << "id.roadId: " << id.roadId << std::endl;
	std::cout << "id.sectionIndex: " << id.sectionIndex << std::endl;
	std::cout << "id.laneId: " << id.laneId << std::endl;

	//get end point
	SSD::SimPoint3DVector inputPoints;
	SSD::SimPoint3D endPt;
	inputPoints.push_back(MainVehiclePos);

	if (SimOneAPI::GetWayPoints(MainVehicleId, &WayPoints))
	{
		int waySize = WayPoints.wayPointsSize;
		endPt.x = WayPoints.wayPoints[waySize - 1].posX;
		endPt.y = WayPoints.wayPoints[waySize - 1].posY;
		endPt.z = 0;
		inputPoints.push_back(endPt);
	}

	SSD::SimVector<int> indexOfValidPoints;
	SSD::SimVector<long> roadIdList;
	SimOneAPI::Navigate(inputPoints, indexOfValidPoints, roadIdList);
	HDMapStandalone::MSignal light = GetTargetLight(id, laneId, roadIdList);
	SSD::SimPoint3D stopLine = GetTragetStopLine(light, laneId);
	SSD::SimString targetSuccessorLaneId = GetTargetSuccessorLane(successorLaneNameList, light.validities);
	
	while (1) {
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop) {
			break;
		}

		int frame = SimOneAPI::Wait();
		if (SimOneAPI::GetGps(MainVehicleId,&Gps))
		{
			float steering = (float)UtilDriver::calculateSteering(targetPath, &Gps);

			SSD::SimPoint3D vehiclePos(Gps.posX, Gps.posY, Gps.posZ);
			if (IsInSafeDistance(vehiclePos, stopLine))
			{
				UtilDriver::setDriver(Gps.timestamp, 0.3f, 0.f, steering);
			}
			else
			{
				if (IsGreenLight(light.id, laneId, light))
				{
					
					GetSuccessorTargetPath(vehiclePos, laneId, targetSuccessorLaneId, stopLine, targetPath);
					UtilDriver::setDriver(Gps.timestamp, 0.1f, 1.f, steering);
				}
				else
				{
					std::cout << "start brake" << std::endl;
					UtilDriver::setDriver(Gps.timestamp, 0, 1, 0);
				}
			}
			SimOneAPI::NextFrame(frame);
		}

	}
}
