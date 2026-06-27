
#include "SimOneServiceAPI.h"
#include "SimOneSensorAPI.h"
#include "SimOneHDMapAPI.h"
#include "SimOneEvaluationAPI.h"
#include "SSD/SimPoint3D.h"
#include "UtilDriver.h"
#include "UtilMath.h"
#include "C:\sim-one\SimOneAPI\Tutorial\HDMap\include\SampleGetNearMostLane.h"
#include "C:\sim-one\SimOneAPI\Tutorial\HDMap\include\SampleGetLaneST.h"

#include <iostream>
#include <memory>

//Main function
int main()
{
	bool inAEBState = false;
	bool EnterScenario03 = false;//判断是否进入场景03
	//场景03：测试车辆以 30km/h 的车速沿车道中间匀速行驶，前方行人在距车辆横向 25m，纵向 4m 处以5km/h 的速度横穿马路。
	int timeout = 20;
	bool isSimOneInitialized = false;
	const char* MainVehicleId = "0";
	bool isJoinTimeLoop = true;
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);
	SimOneAPI::SetDriverName(MainVehicleId, "LKA");
	SimOneAPI::InitEvaluationServiceWithLocalData(MainVehicleId);//评价函数
	//加载地图
	while (true) {
		if (SimOneAPI::LoadHDMap(timeout)) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loaded");
			break;
		}
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loading...");
	}

	//输入点的容器
	SSD::SimPoint3DVector inputPoints;
	//存储车道信息的指针
	std::unique_ptr<SimOne_Data_WayPoints> pWayPoints = std::make_unique<SimOne_Data_WayPoints>();

	//主车路点数据
	if (SimOneAPI::GetWayPoints(MainVehicleId, pWayPoints.get()))
	{
		// 提取主车道点的坐标并添加到输入点容器中
		for (size_t i = 0; i < pWayPoints->wayPointsSize; ++i) {
			SSD::SimPoint3D inputWayPoints(pWayPoints->wayPoints[i].posX, pWayPoints->wayPoints[i].posY, 0);
			inputPoints.push_back(inputWayPoints);
		}
	}
	else {
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Get mainVehicle wayPoints failed");
		return -1;
	}

	//目标路径的容器
	SSD::SimPoint3DVector targetPath;
	if (pWayPoints->wayPointsSize >= 2) {
		SSD::SimVector<int> indexOfValidPoints;

		//生成主车路线
		if (!SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath)) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Generate mainVehicle route failed");
			return -1;
		}
	}
	else if (pWayPoints->wayPointsSize == 1) {
		//只有一个路点，获取最近的车道并初始化路线
		SSD::SimString laneIdInit = SampleGetNearMostLane(inputPoints[0]);
		HDMapStandalone::MLaneInfo laneInfoInit;
		if (!SimOneAPI::GetLaneSample(laneIdInit, laneInfoInit)) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Generate mainVehicle initial route failed");
			return -1;
		}
		else {
			targetPath = laneInfoInit.centerLine;
		}
	}
	else {
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Number of wayPoints is zero");
		return -1;
	}

	while (true) {
		int frame = SimOneAPI::Wait();

		// 根据驻车停止状态进入评价函数
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "SimOneAPI::GetCaseRunStatus() = %f", SimOneAPI::GetCaseRunStatus());
			SimOneAPI::SaveEvaluationRecord();
			break;
		}

		//获取GPS信息
		std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
		if (!SimOneAPI::GetGps(MainVehicleId, pGps.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch GPS failed");
		}

		//获取障碍物数据
		std::unique_ptr<SimOne_Data_Obstacle> pObstacle = std::make_unique<SimOne_Data_Obstacle>();
		if (!SimOneAPI::GetGroundTruth(MainVehicleId, pObstacle.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch obstacle failed");
		}

		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Running) {
			if (!isSimOneInitialized) {
				SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initialized!");
				isSimOneInitialized = true;
			}

			//获取主车位置和速度
			SSD::SimPoint3D mainVehiclePos(pGps->posX, pGps->posY, pGps->posZ);
			double mainVehicleSpeed = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ);

			//初始化距离和障碍物索引
			double minDistance = std::numeric_limits<double>::max();
			int potentialObstacleIndex = pObstacle->obstacleSize;
			SSD::SimString mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos);
			SSD::SimString potentialObstacleLaneId = "";
			for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {
				//获取障碍位置和车道
				SSD::SimPoint3D obstaclePos(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, pObstacle->obstacle[i].posZ);
				SSD::SimString obstacleLaneId = SampleGetNearMostLane(obstaclePos);
				//通过地图坐标判断是否进入场景03
				if (pObstacle->obstacle[i].posY < -6.5)
					EnterScenario03 = true;
				
				//如果主车道和障碍物在同一车道
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
				// 获取车道ST坐标
				SampleGetLaneST(potentialObstacleLaneId, potentialObstaclePos, sObstalce, tObstacle);
				SampleGetLaneST(mainVehicleLaneId, mainVehiclePos, sMainVehicle, tMainVehicle);

				isObstalceBehind = !(sMainVehicle >= sObstalce);
			}

			std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();

			// 使用SimOneDriver控制主车
			SimOneAPI::GetDriverControl(MainVehicleId, pControl.get());

			// Control mainVehicle without SimOneDriver
			/*pControl->throttle = 0.12f;  // 油门踏板位置，设置为0.12表示轻踩油门
			pControl->brake = 0.f;        // 制动踏板位置，设置为0表示没有刹车
			pControl->steering = 0.f;     // 方向盘转角，设置为0表示没有方向盘转动
			pControl->handbrake = false;  // 手刹状态，设置为false表示手刹未拉起
			pControl->isManualGear = false;  // 手动挡标志，设置为false表示不使用手动挡
			pControl->gear = static_cast<ESimOne_Gear_Mode>(1);  // 档位设置，这里将档位设置为1
			*/


			// 如果有障碍物
			if (isObstalceBehind) {
				double defaultDistance = obstacleSpeed > 2 ? 5.0f : 1.0f;
				//动态障碍物安全距离5，静态障碍物安全距离1（动静态阈值2）
				double timeToCollision = std::abs((minDistance - defaultDistance)) / (obstacleSpeed - mainVehicleSpeed);
				double defautlTimeToCollision = (obstacleSpeed < 2 && obstacleSpeed > 1) ? 3.0f : 1.15f;
				//速度1-2m/s的障碍（行人）刹车更早，安全距离更长
				if (-timeToCollision < defautlTimeToCollision && timeToCollision < 0) {
					inAEBState = true;
					//pControl->brake = (float)(mainVehicleSpeed * 6.3 * 0.95 + 0.20);
				}

				if (inAEBState) {
					pControl->brake = 1,0;//刹车
					//油门力度
					pControl->throttle = 0.;
				}
			}
			if(obstacleSpeed == 0 && EnterScenario03) {
				pControl->brake = 0.;//刹车
				pControl->throttle = 0.9;
			}//行人完全通过后车辆重新启动
			
			// 转向
			//double steering = UtilDriver::calculateSteering(targetPath, pGps.get());
			//pControl->steering = (float)steering;
			SimOneAPI::SetDrive(MainVehicleId, pControl.get());
		}
		else {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initializing...");
		}
		SimOneAPI::NextFrame(frame);
	}
	return 0;
}
