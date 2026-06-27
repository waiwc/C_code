
#include "SimOneServiceAPI.h"
#include "SimOneSensorAPI.h"
#include "SimOneHDMapAPI.h"
#include "SSD/SimPoint3D.h"
#include "UtilDriver.h"
#include "UtilMath.h"
#include "SimOneEvaluationAPI.h"
#include "SampleGetNearMostLane.h"
#include "SampleGetLaneST.h"

#include <iostream>
#include <memory>

//Main function
//
int main()
{
	bool inAEBState = false;
	int timeout = 20;
	bool isSimOneInitialized = false;
	const char* MainVehicleId ="0";
	bool isJoinTimeLoop = true;
	SimOneAPI::InitEvaluationServiceWithLocalData(MainVehicleId);
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);
	SimOneAPI::SetDriverName(MainVehicleId, "LKA");
	SSD::SimPoint3DVector inputPoints;
	SSD::SimPoint3DVector targetPath;
	
	    // 地图加载循环
    while (true)
    {
        int timeout = 20;
        if (!SimOneAPI::LoadHDMap(timeout))
        {
            SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loaded");
            continue;
        }
        SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loading...");

        // 获取路径点
        std::unique_ptr<SimOne_Data_WayPoints> pWayPoints = std::make_unique<SimOne_Data_WayPoints>();
		if (SimOneAPI::GetWayPoints(MainVehicleId, pWayPoints.get()))
		{
			for (size_t i = 0; i < pWayPoints->wayPointsSize; ++i) {
				SSD::SimPoint3D inputWayPoints(pWayPoints->wayPoints[i].posX, pWayPoints->wayPoints[i].posY, 0);
				inputPoints.push_back(inputWayPoints);
			}
		}
		else {
            SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Get mainVehicle wayPoints failed");
            continue;
        }

        // 生成路线
        if (pWayPoints->wayPointsSize >= 2)
        {
            SSD::SimVector<int> indexOfValidPoints;
            if (!SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath))
            {
                SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error,
                                     "Generate mainVehicle route failed");
                continue;
            }
        }
        else if (pWayPoints->wayPointsSize == 1)
        {
            SSD::SimString laneIdInit = SampleGetNearMostLane(inputPoints[0]);
            HDMapStandalone::MLaneInfo laneInfoInit;
            if (!SimOneAPI::GetLaneSample(laneIdInit, laneInfoInit))
            {
                SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error,
                                     "Generate mainVehicle initial route failed");
                continue;
            }
            else
            {
                targetPath = laneInfoInit.centerLine;
            }
        }
        else
        {
            SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Number of wayPoints is zero");
            continue;
        }
        break;
    }
	
	while (true) {
		int frame = SimOneAPI::Wait();

		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop) {
			SimOneAPI::SaveEvaluationRecord();
			break;
		}

		std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
		if (!SimOneAPI::GetGps(MainVehicleId,pGps.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch GPS failed");
		}

		std::unique_ptr<SimOne_Data_Obstacle> pObstacle = std::make_unique<SimOne_Data_Obstacle>();
		if (!SimOneAPI::GetGroundTruth(MainVehicleId,pObstacle.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch obstacle failed");
		}

		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Running) {
			if (!isSimOneInitialized) {
				SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initialized!");
				isSimOneInitialized = true;
			}

			SSD::SimPoint3D mainVehiclePos(pGps->posX, pGps->posY, pGps->posZ);
			double mainVehicleSpeed = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ);

			double minDistance = std::numeric_limits<double>::max();
			int potentialObstacleIndex = pObstacle->obstacleSize;
			SSD::SimString mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos);
			SSD::SimString potentialObstacleLaneId = "";
			for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {
				SSD::SimPoint3D obstaclePos(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, pObstacle->obstacle[i].posZ);
				SSD::SimString obstacleLaneId = SampleGetNearMostLane(obstaclePos);
				if (mainVehicleLaneId == obstacleLaneId) {
					double obstacleDistance = UtilMath::planarDistance(mainVehiclePos, obstaclePos);

					if (obstacleDistance < minDistance) {
						minDistance = obstacleDistance;
						potentialObstacleIndex = (int)i;
						potentialObstacleLaneId = obstacleLaneId;
					}
				}
			}

			auto& potentialObstacle = pObstacle->obstacle[potentialObstacleIndex];
			double obstacleSpeed = UtilMath::calculateSpeed(potentialObstacle.velX, potentialObstacle.velY, potentialObstacle.velZ);


			SSD::SimPoint3D potentialObstaclePos(potentialObstacle.posX, potentialObstacle.posY, potentialObstacle.posZ);
			double sObstalce = 0.;
			double tObstacle = 0.;

			double sMainVehicle = 0.;
			double tMainVehicle = 0.;

			bool isObstalceBehind = false;
			if (!potentialObstacleLaneId.Empty()) {

				SampleGetLaneST(potentialObstacleLaneId, potentialObstaclePos, sObstalce, tObstacle);
				SampleGetLaneST(mainVehicleLaneId, mainVehiclePos, sMainVehicle, tMainVehicle);

                isObstalceBehind = !(sMainVehicle >= sObstalce);
			}

			std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();

			// Control mainVehicle with SimOneDriver
			SimOneAPI::GetDriverControl(MainVehicleId, pControl.get());

			// Control mainVehicle without SimOneDriver
			/*pControl->throttle = 0.12f;
			pControl->brake = 0.f;
			pControl->steering = 0.f;
			pControl->handbrake = false;
			pControl->isManualGear = false;
			pControl->gear = static_cast<ESimOne_Gear_Mode>(1);*/

			if (isObstalceBehind) {
				double defaultDistance = 10.;
				double timeToCollision = std::abs((minDistance - defaultDistance)) / (obstacleSpeed - mainVehicleSpeed);
				double defautlTimeToCollision = 3.4;
				if (-timeToCollision < defautlTimeToCollision && timeToCollision < 0) {
					inAEBState = true;
					pControl->brake = (float)(mainVehicleSpeed * 3.6 * 0.65 + 0.20);
				}

				if (inAEBState) {
					pControl->throttle = 0.f;
				}
			}
			double steering = UtilDriver::calculateSteering(targetPath, pGps.get());
			pControl->steering = (float)steering;
			SimOneAPI::SetDrive(MainVehicleId, pControl.get());
		}
		else {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initializing...");
		}

		SimOneAPI::NextFrame(frame);
	}
	return 0;
}
