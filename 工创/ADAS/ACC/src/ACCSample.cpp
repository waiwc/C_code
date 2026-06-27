#include "SimOneNetAPI.h"
#include "SSD/SimPoint3D.h"
#include "util/UtilMath.h"
#include "utilStartSimOneNode.h"
#include "PID.hpp"
#include "../HDMap/include/SampleGetNearMostLane.h"
#include "../HDMap/include/SampleGetLaneST.h"

#include <memory>
#include <limits>
#include <iostream>

//Main function
//
int main()
{
	bool inAEBState = false;
	bool isSimOneInitialized = false;
	StartSimOne::WaitSimOneIsOk(true);
	SimOneSM::SetDriverName(0, "ACC");

	// load HdMap
	int timeout = 20;
	while (true) {
		if (SimOneSM::LoadHDMap(timeout)) {
			std::cout << "HDMap Information Loaded" << std::endl;
			break;
		}
		std::cout << "HDMap Information Loading..." << std::endl;
	}

	while (true) {
		int frame = SimOneAPI::Wait(); 

		if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Stop) {
			break;
		}

		std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
		if (!SimOneSM::GetGps(0, pGps.get())) {
			std::cout << "Fetch GPS failed" << std::endl;
		}

		std::unique_ptr<SimOne_Data_Obstacle> pObstacle = std::make_unique<SimOne_Data_Obstacle>();
		if (!SimOneAPI::GetSimOneGroundTruth(pObstacle.get())) {
			std::cout << "Fetch obstacle failed" << std::endl;
		}

		if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Running && pObstacle->timestamp > 0 && pGps->timestamp > 0) {
			if (!isSimOneInitialized) {
				std::cout << "SimOne Initialized!" << std::endl;
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
			SimOneSM::GetDriverControl(0, pControl.get());

			std::cout << "Command Form SimOneDriver(throttle, brake, steering)" << pControl->throttle << "," <<
				pControl->brake << "," << pControl->steering << std::endl;

			// Control mainVehicle without SimOneDriver
			/*pControl->throttle = 0.5f;
			pControl->brake = 0.f;
			pControl->steering = 0.f;
			pControl->handbrake = false;
			pControl->isManualGear = false;
			pControl->gear = static_cast<EGearMode>(1);*/

			if (isObstalceBehind) {
				double defaultDistance = 10.0;
				double safeDistance = 30.0;
				double timeToCollision = std::abs((minDistance - defaultDistance)) / (obstacleSpeed - mainVehicleSpeed);
				double defaultTimeToCollisionGeneral = 3.6;
				double defaultTimeToCollisionUrgent = 1.8;
			
				if (minDistance < safeDistance) {

					PID PidControl(0.3, 1.8 * minDistance / 20., 0.);

					float command = -(float)(PidControl.pidControl(safeDistance, minDistance) / (0.5f * 0.05f * 0.05f));

					pControl->throttle = command > 0 ? command : 0.f;
					pControl->brake = command > 0 ? 0.f : -command;

					inAEBState = false;
					if (-timeToCollision < defaultTimeToCollisionGeneral && timeToCollision < 0) {
						inAEBState = true;
						pControl->brake = (float)(mainVehicleSpeed * 3.6 * 1.0 + 0.20);
						if (-timeToCollision < defaultTimeToCollisionUrgent) {
							pControl->brake = (float)(mainVehicleSpeed * 3.6 * 1.2 + 0.20);
						}
					}
					if (inAEBState) {
						pControl->throttle = 0.f;
					}
				}				
			}
			SimOneSM::SetDrive(0, pControl.get());
		}
		else {
			std::cout << "SimOne Initializing..." << std::endl;
		}		
		SimOneAPI::NextFrame(frame);
	}
	return 0;
}
