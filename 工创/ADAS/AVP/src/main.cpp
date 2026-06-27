#include <iostream>
#include <memory>
#include <vector>
#include <fstream>
#include <thread>
#include <chrono>
#include <fstream>

#include "SimOneEvaluationAPI.h"
#include "SimOneServiceAPI.h"
#include "SimOneHDMapAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneSensorAPI.h"
#include "UtilMath.hpp"
#include "UtilUnit.hpp"
#include "UtilDriver.hpp"
#include "AVPLog.hpp"
#include "AVPPlanner.hpp"
#include "hdmap/SampleGetDistanceToLaneBoundary.h"

enum AVPStatus
{
	eApproaching,
	eForwarding,
	eBrakingPrep,
	eReversing,
	eFinishing,
	eWaiting,
	eLeavingPre,
	eLeaving,
	estraightly,
	eEnded,
	eUnknown
};
enum APAStatus
{
	Forwarding,
	BrakingPrep,
	Reversing,
	Finishing,
	Waiting,
	Ended,
	Leaving,
	End
};
// 侧方 = 0，垂直 = 1，斜列 = 2
enum ParkingType
{
	kParallel = 0,
	kPerpendicular = 1,
	kAngled = 2
};//车位类型

/*!
* @function FindTargetParkingSpace
* @brief Find the target parking space
* @param
*   parkingSpaces: List of all parking spaces
@param
*   obstacles: Pointer of information of all the obstacles
* @param[out]
*   targetParkingSpace: Found target parking space
*/
/*!
* @brief   找可用车位（循环内实时获取障碍物，多接口测试并完整日志输出）
* @param   mainVehicleId   主车 ID（如 "0"）
* @param   parkingSpaces   地图中全部车位列表
* @param[out] targetParkingSpace  选出的目标车位
* @return  true=找到可用车位，false=全部被占
*/
bool FindTargetParkingSpace(
	const SSD::SimVector<HDMapStandalone::MParkingSpace>& parkingSpaces,
    const char* mainVehicleId,
    HDMapStandalone::MParkingSpace& targetParkingSpace)
{
    SimOne_Data_Obstacle           gt{};
    SimOne_Data_SensorDetections   det{};
    SimOne_Data_RadarDetection     radar{};

    // 持续查询，直到找出空余车位
    while (true)
    {
        /* 1. 取三类障碍物数据，全部打印出来做对照 ---------------------------*/
        if (SimOneAPI::GetGroundTruth(mainVehicleId, &gt))
        {
            std::cout << "[GT] obstacleSize = " << gt.obstacleSize << std::endl;
            for (int i = 0; i < gt.obstacleSize; ++i)
            {
                const auto& o = gt.obstacle[i];
                std::cout << "  id=" << o.id
                          << " pos=(" << o.posX << ',' << o.posY << ")"
                          << " heading=" << o.oriZ << std::endl;  // 用 oriZ！
            }
        }

        if (SimOneAPI::GetSensorDetections(mainVehicleId, "PerceptionPerfect0", &det))
        {
            std::cout << "[Sensor] objectSize = " << det.objectSize << std::endl;
            for (int i = 0; i < det.objectSize; ++i)
                std::cout << "  det id=" << det.objects[i].id << '\n';
        }

        if (SimOneAPI::GetRadarDetections(mainVehicleId, "RAD_F0", &radar))
        {
            std::cout << "[Radar] detectNum = " << radar.detectNum << std::endl;
        }

        /* 2. 用最新 ground-truth 判断车位是否被占 ---------------------------*/
        std::vector<size_t> freeIdx;
        for (size_t i = 0; i < parkingSpaces.size(); ++i)
        {
            const auto& ps = parkingSpaces[i];
            bool occupied = false;
            for (int j = 0; j < gt.obstacleSize; ++j)
            {
                const auto& o = gt.obstacle[j];
                SSD::SimPoint3D pos(o.posX, o.posY, o.posZ);
                if (UtilMath::InRectangleMargin(pos,
                                                ps.boundaryKnots[0],
                                                ps.boundaryKnots[2]))
                {
                    occupied = true;
                    break;
                }
            }
            if (!occupied) freeIdx.push_back(i);
        }

        if (!freeIdx.empty())
        {
            targetParkingSpace = parkingSpaces[freeIdx[0]];
            std::cout << ">>> 空余车位索引: " << freeIdx[0] << std::endl;
            break;   // 找到了就退出循环
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
	return true;
}
/*!
* @brief  重排 knots，并返回泊位类型  
*         0 = Parallel(侧方) , 1 = Perpendicular(垂直) , 2 = Angled(斜列)
* @param  space      [in/out] MParkingSpace，重排后顺序为 LF, RF, RR, LR
* @param  vehYawRad  车辆当前朝向（rad, 直接 gps->oriZ）
*/
ParkingType RearrangeKnots(HDMapStandalone::MParkingSpace& space,
                           double vehYawRad)
{
    /* ---------- 1. 重排四个角点 ---------- */
    const int N = 4;
    SSD::SimPoint3D tmp[4];

    SSD::SimPoint2D heading{ space.heading.x, space.heading.y };
    const double thr = UtilUnit::DegreeToRad(10.0);

    int startIdx = 0;
    for (int i = 0; i < N; ++i)
    {
        SSD::SimPoint2D dir = { space.boundaryKnots[(i+1)%4].x - space.boundaryKnots[i].x,
                                space.boundaryKnots[(i+1)%4].y - space.boundaryKnots[i].y };
        if (dir.x==0 && dir.y==0) continue;
        double ang = UtilMath::Angle(heading, dir);
        if (ang > 0.0 && ang < thr) { startIdx = (i + 2) % 4; break; }
    }
    for (int j=0;j<N;++j) tmp[j] = space.boundaryKnots[(startIdx+j)%4];
    for (int j=0;j<N;++j) space.boundaryKnots[j] = tmp[j];
    // 现在 [0]=LF  [1]=RF  [2]=RR  [3]=LR

    /* ---------- 2. 取长边作为车位纵向 ---------- */
    const auto& lf = space.boundaryKnots[0];
    const auto& rf = space.boundaryKnots[1];
    const auto& lr = space.boundaryKnots[3];

    // front 边向量与 side 边向量
    double fx = rf.x - lf.x, fy = rf.y - lf.y;   // 宽
    double sx = lr.x - lf.x, sy = lr.y - lf.y;   // 长

    double lenF2 = fx*fx + fy*fy;
    double lenS2 = sx*sx + sy*sy;

    // 取更长的那条作为“车位纵向”
    double slotYaw = 0.0;
    if (lenS2 >= lenF2)          // side 更长 → LF→LR
        slotYaw = std::atan2(sy, sx);
    else                         // front 更长 → LF→RF (少见，但兼容)
        slotYaw = std::atan2(fy, fx);

    /* ---------- 3. 与车辆朝向比较 ---------- */
    double dYaw = std::fabs(slotYaw - vehYawRad);
    if (dYaw > M_PI) dYaw = 2*M_PI - dYaw;   // 折叠到 [0,π]
    if (dYaw > M_PI_2) dYaw = M_PI - dYaw;   // 折叠到 [0,π/2]

    const double PAR_THR  = UtilUnit::DegreeToRad(15.0);
    const double PERP_THR = UtilUnit::DegreeToRad(15.0);

    if (dYaw < PAR_THR)                       return kParallel;
    if (std::fabs(dYaw - M_PI_2) < PERP_THR) return kPerpendicular;
    return kAngled;
}

/*!
* @function InParkingSpace
* @brief Determine if a vehicle is inside a parking space
* @param
*   pos: Position of the vehicle (its rear center)
* @param
*   theta: Heading of the vehicle in radians
* @param
*   length: Length of the vehicle
* @param
*   width: Width of the vehicle
* @param
*   space: Interested parking space
* @return
*   True if any of the vehicle's vertices is inside the parking space; false otherwise
*/
bool InParkingSpace(const SSD::SimPoint3D& pos, const double theta, const double length, const double width, const HDMapStandalone::MParkingSpace& space)
{
	SSD::SimPoint3D p1(pos.x + width / 2. * cos(theta) + length / 2. * sin(theta),
		pos.y + width / 2. * sin(theta) - length / 2. * cos(theta), pos.z);
	SSD::SimPoint3D p2(pos.x + width / 2. * cos(theta) - length / 2. * sin(theta),
		pos.y + width / 2. * sin(theta) + length / 2. * cos(theta), pos.z);
	SSD::SimPoint3D p3(pos.x - width / 2. * cos(theta) - length / 2. * sin(theta),
		pos.y - width / 2. * sin(theta) + length / 2. * cos(theta), pos.z);
	SSD::SimPoint3D p4(pos.x - width / 2. * cos(theta) + length / 2. * sin(theta),
		pos.y - width / 2. * sin(theta) - length / 2. * cos(theta), pos.z);

	return UtilMath::InRectangleMargin(p1, space.boundaryKnots[0], space.boundaryKnots[2]) || UtilMath::InRectangleMargin(p2, space.boundaryKnots[0], space.boundaryKnots[2]) ||
		UtilMath::InRectangleMargin(p3, space.boundaryKnots[0], space.boundaryKnots[2]) || UtilMath::InRectangleMargin(p4, space.boundaryKnots[0], space.boundaryKnots[2]);
}

int main(int argc, char* argv[])
{
	std::unique_ptr<SimOne_Data_Gps> gpsPtr = std::make_unique<SimOne_Data_Gps>();
	std::unique_ptr<SimOne_Data_Obstacle> obstaclesPtr = std::make_unique<SimOne_Data_Obstacle>();
	std::unique_ptr<SimOne_Data_WayPoints> wayPointsPtr = std::make_unique<SimOne_Data_WayPoints>();

	bool leaveAfterParked = true;
	bool isJoinTimeLoop = true;
	const char* MainVehicleId = "0";
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);
	SimOneAPI::InitEvaluationServiceWithLocalData(MainVehicleId);
	SimOneAPI::SetDriverName(0, "AVP");

	int timeoutSecond = 20;
	if (!SimOneAPI::LoadHDMap(timeoutSecond))
	{
		std::cout << "Failed to load map!" << std::endl;
		return 0;
	}

	SSD::SimPoint3D startPoint;
	if (SimOneAPI::GetGps(0, gpsPtr.get()))
	{
		startPoint.x = gpsPtr->posX;
		startPoint.y = gpsPtr->posY;
		startPoint.z = gpsPtr->posZ;
	}
	else
	{
		std::cout << "Fetch GPS failed" << std::endl;
	}


	/* 2. Get parking spaces */
	SSD::SimVector<HDMapStandalone::MParkingSpace> parkingSpaces;
	SimOneAPI::GetParkingSpaceList(parkingSpaces);

	for (size_t i = 0; i < parkingSpaces.size(); ++i)
	{
		auto& parkingSpace = parkingSpaces[i];
		auto& vertices = parkingSpace.boundaryKnots;
		std::cout << "ParkingSpace[" << i << "]: " << std::endl;
		for (size_t j = 0; j < 4; ++j)
		{
			std::cout << "[" << vertices[j].x << ", " << vertices[j].y << "]" << std::endl;
		}
	}
	std::cout << std::endl;


	/* 3. Get obstacles */
	SimOneAPI::GetGroundTruth(MainVehicleId, obstaclesPtr.get());
	for (size_t i = 0; i < obstaclesPtr->obstacleSize; ++i)
	{
		std::cout << "Obstacles[" << i << "]: " << std::endl;
		auto& obstacle = obstaclesPtr->obstacle[i];
		std::cout << "[" << obstacle.posX << ", " << obstacle.posY << "]" << std::endl;
	}
	std::cout << std::endl;


	/* 4. Determine target parking space */
	HDMapStandalone::MParkingSpace targetParkingSpace;
	FindTargetParkingSpace(parkingSpaces, MainVehicleId, targetParkingSpace);

	// rearrange boundaryKnots of the target parking space
	std::unique_ptr<SimOne_Data_Gps> gps = std::make_unique<SimOne_Data_Gps>();
SimOneAPI::GetGps(MainVehicleId, gps.get());

ParkingType type = RearrangeKnots(targetParkingSpace, gps->oriZ);
	std::cout << "Rearranged parking space:" << std::endl;

	for (size_t i = 0; i < targetParkingSpace.boundaryKnots.size(); ++i)
	{
		std::cout << "[" << targetParkingSpace.boundaryKnots[i].x << ", " << targetParkingSpace.boundaryKnots[i].y << "]" << std::endl;
#if AVP_LOG
		AVPLog::getInstance().addData(targetParkingSpace.boundaryKnots[i].x);
		AVPLog::getInstance().addData(targetParkingSpace.boundaryKnots[i].y);
#endif // AVP_LOG
	}
	std::cout << std::endl;
#if AVP_LOG
	AVPLog::getInstance().addNewLine();
#endif // AVP_LOG


	/* 5. Get terminal point */
	while (!SimOneAPI::GetWayPoints(MainVehicleId, wayPointsPtr.get()))
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	auto lastWayPoint = wayPointsPtr->wayPoints[wayPointsPtr->wayPointsSize - 1];
	SSD::SimPoint3D terminalPoint{ lastWayPoint.posX, lastWayPoint.posY, 0. };

	/* 6. Plan */
	VehicleParam veh;
	veh.Lr = 0.88;
	veh.Lf = 1.;
	veh.L = 2.9187;

	AVPPlanner planner(targetParkingSpace, startPoint, terminalPoint, veh, leaveAfterParked);
	planner.plan(type);


	/* 7. Control */
	AVPStatus status = AVPStatus::eForwarding;
	APAStatus APAstatus = APAStatus::Forwarding;

	bool brakingPrepStarted = false;
	bool reversingStarted = false;
	bool finishingStarted = false;
	bool waitingStarted = false;
	bool leavepre = false;
	bool reverseside = false;
	double brakingPrepTime = 1.;
	double waitingTimeSecond = 10.;
	auto startTime = std::chrono::system_clock::now();
	auto Time = std::chrono::system_clock::now();
	auto reversetime = std::chrono::system_clock::now();
	double headingErrorThresholdDegree = 1.;
	double parkingEndDistanceThreshold = 0.5;
	double leavingEndDistanceThreshold = 0.5;

	while (true)
	{
		int frame = SimOneAPI::Wait();

		// 根据驻车停止状态进入评价函数
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop) {
			SimOneAPI::SaveEvaluationRecord();
			break;
		}

		if (!SimOneAPI::GetGps(MainVehicleId, gpsPtr.get()))
		{
			std::cout << "Fetch GPS failed" << std::endl;;
		}
		std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
		if (!SimOneAPI::GetGps(MainVehicleId, pGps.get())) {
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch GPS failed");
		}
		double yaw = pGps->oriZ;
		SSD::SimPoint3D mainVehiclePos(pGps->posX, pGps->posY, pGps->posZ);
		int lanedisback;
		SampleGetDistanceToLaneBoundary(mainVehiclePos , &lanedisback);
		//if(lanedisback)
		//	status = AVPStatus::eBrakingPrep;
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "lanedisbacke : %d", lanedisback);
		
#if AVP_LOG
		AVPLog::getInstance().addData(gpsPtr->posX);
		AVPLog::getInstance().addData(gpsPtr->posY);
#endif // AVP_LOG
		int stop_position_x = 0;
		if (type == kPerpendicular) {
			stop_position_x = -2;
			if (status == AVPStatus::eForwarding) // approaching the parking space
			{
				if (//UtilMath::PlanarDistance({ gpsPtr->posX, gpsPtr->posY, gpsPtr->posZ }, planner.ForwardTrajectory().back()) < 0.05
				(gpsPtr->posX > -6)
					|| lanedisback)
				{
					UtilDriver::SetControl(gpsPtr->timestamp, 0., 1., 0., ESimOne_Gear_Mode_Neutral);
					status = AVPStatus::eBrakingPrep;
					std::cout << "Reached reverse point!" << std::endl;
				}
				else {
					double speedKmH = UtilUnit::MsToKmH(UtilMath::Length({ gpsPtr->velX, gpsPtr->velY, gpsPtr->velZ }));
					double throttle = speedKmH < 30 ? 1 : 0;
					double brake = speedKmH > 30 ? 0.5 : 0;
					// double throttle = speedKmH < 3 ? 0.03 : 0;
					// double brake = speedKmH > 20 ? 0.03 : 0;
					double steering = UtilDriver::CalculateSteering(planner.ForwardTrajectory(), gpsPtr.get());
					//steering = steering > 0.3 ? 0.3 : steering;
					SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "steering: %lf", steering);
					UtilDriver::SetControl(gpsPtr->timestamp, throttle, brake, steering);
				}
			}
			else if (status == AVPStatus::eBrakingPrep) // coming to a full stop
			{
				double throttle = 0;
				double brake = 1;
				UtilDriver::SetControl(gpsPtr->timestamp, throttle, brake, 0);
				if (!brakingPrepStarted)
				{
					startTime = std::chrono::system_clock::now();
					brakingPrepStarted = true;
				}
				else
				{
					// wait for a total of 1 second
					std::chrono::duration<double> currentBrakingPrepTime = std::chrono::system_clock::now() - startTime;
					if (currentBrakingPrepTime.count() > brakingPrepTime)
					{
						status = AVPStatus::eReversing;
					}
				}
				UtilDriver::SetControl(gpsPtr->timestamp, 0., 1., 0., ESimOne_Gear_Mode_Neutral);
			}
			else if (status == AVPStatus::eReversing) // reversing towards the parking space
			{
				if //(UtilMath::CloseEnough(std::atan2(targetParkingSpace.heading.y, targetParkingSpace.heading.x),
					//(double)gpsPtr->oriZ, UtilUnit::DegreeToRad(headingErrorThresholdDegree)))
					(lanedisback)
				{
					status = AVPStatus::eFinishing;
					std::cout << "Start reversing straightly!" << std::endl;
				}
				else
				{
					double speedKmH = UtilUnit::MsToKmH(UtilMath::Length({ gpsPtr->velX, gpsPtr->velY, gpsPtr->velZ }));
					double throttle, brake;
					if (speedKmH < 3 && !reversingStarted)
					{
						throttle = 0.03f;
						brake = 0.f;
					}
					else
					{
						reversingStarted = true;
						throttle = 0.f;
						brake = 0.02f;
					}

					double steering = UtilDriver::CalculateSteering(planner.ReverseTrajectory(), gpsPtr.get());
					steering = yaw > 1.55 ? 0 : steering;
					UtilDriver::SetControl(gpsPtr->timestamp, throttle, brake, steering, ESimOne_Gear_Mode_Reverse);
				}
			}
			else if (status == AVPStatus::eFinishing) // finishing
			{
				//double endDistance = UtilMath::Distance({ gpsPtr->posX, gpsPtr->posY }, planner.ParkingEndPoint());
				//if (endDistance < parkingEndDistanceThreshold)
				//{
				std::cout << "Finished parking!" << std::endl;
				UtilDriver::SetControl(gpsPtr->timestamp, 0., 1., 0., ESimOne_Gear_Mode_Neutral);
				if (leaveAfterParked)
				{
					status = AVPStatus::eWaiting;
				}
				else
				{
					status = AVPStatus::eEnded;
					break;
				}
				//}
				// let the vehicle slide
				//UtilDriver::SetControl(gpsPtr->timestamp, 0., 0., 0., ESimOne_Gear_Mode_Reverse);
			}
			else if (status == AVPStatus::eWaiting)
			{
				if (!waitingStarted)
				{
					startTime = std::chrono::system_clock::now();
					waitingStarted = true;
				}
				else
				{
					// wait for a total of waitingTimeSecond
					std::chrono::duration<double> currentWaitingTimeSecond = std::chrono::system_clock::now() - startTime;
					if (currentWaitingTimeSecond.count() > waitingTimeSecond)
					{
						status = AVPStatus::eLeavingPre;
						std::cout << "Finished waiting for " << waitingTimeSecond << " second!" << std::endl;
					}
				}
				UtilDriver::SetControl(gpsPtr->timestamp, 0., 1., 0., ESimOne_Gear_Mode_Neutral);
			}
			else if (status == AVPStatus::eLeavingPre)
			{
				if (!leavepre)
				{
					Time = std::chrono::system_clock::now();
					leavepre = true;
				}
				else
				{
					// wait for a total of waitingTimeSecond
					std::chrono::duration<double> TimeSecond = std::chrono::system_clock::now() - Time;
					if (TimeSecond.count() > 2)
					{
						status = AVPStatus::eLeaving;
					}
				}
				UtilDriver::SetControl(gpsPtr->timestamp, 0.05, 0., 0);
			}
			else if (status == AVPStatus::eLeaving)
			{
				double endDistance = UtilMath::PlanarDistance({ gpsPtr->posX, gpsPtr->posY, 0. }, planner.LeavingTrajectory().back());
				if (endDistance < leavingEndDistanceThreshold)
				{
					status = AVPStatus::eEnded;
					std::cout << "Finished leaving!" << std::endl;
					UtilDriver::SetControl(gpsPtr->timestamp, 0., 1., 0., ESimOne_Gear_Mode_Neutral);
				}
				else
				{
					double steering = UtilDriver::CalculateSteering(planner.LeavingTrajectory(), gpsPtr.get());
					UtilDriver::SetControl(gpsPtr->timestamp, 0.05, 0., 1);
					if (abs(yaw) < 0.01)
					{
						status = AVPStatus::estraightly;
					}
				}
			}
			else if (status == AVPStatus::estraightly) {
				UtilDriver::SetControl(gpsPtr->timestamp, 1, 0., 0);
			}

		}
		else if (type == kParallel)
		{
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "侧方位停车");
			stop_position_x = 11;
			if (APAstatus == APAStatus::Forwarding) // approaching the parking space
			{
				if (//UtilMath::PlanarDistance({ gpsPtr->posX, gpsPtr->posY, gpsPtr->posZ }, planner.ForwardTrajectory().back()) < 0.05
					gpsPtr->posX > stop_position_x
					|| lanedisback)
				{
					UtilDriver::SetControl(gpsPtr->timestamp, 0., 1., 0., ESimOne_Gear_Mode_Neutral);
					APAstatus = APAStatus::BrakingPrep;
					std::cout << "Reached reverse point!" << std::endl;
				}
				else {
					double speedKmH = UtilUnit::MsToKmH(UtilMath::Length({ gpsPtr->velX, gpsPtr->velY, gpsPtr->velZ }));
					//double throttle = speedKmH < 3 ? 0.03 : 0;
					double brake = speedKmH > 50 ? 1 : 0;
					double steering = UtilDriver::CalculateSteering(planner.ForwardTrajectory(), gpsPtr.get());
					//steering = steering > 0.3 ? 0.3 : steering;
					//SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "steering: %lf", steering);
					UtilDriver::SetControl(gpsPtr->timestamp, 0.4, brake, 0.01);
				}
			}
			else if (APAstatus == APAStatus::BrakingPrep) // coming to a full stop
			{
				if (!brakingPrepStarted)
				{
					startTime = std::chrono::system_clock::now();
					brakingPrepStarted = true;
				}
				else
				{
					// wait for a total of 1 second
					std::chrono::duration<double> currentBrakingPrepTime = std::chrono::system_clock::now() - startTime;
					if (currentBrakingPrepTime.count() > 2)
					{
						APAstatus = APAStatus::Reversing;
					}
				}
				UtilDriver::SetControl(gpsPtr->timestamp, 0., 1., 0., ESimOne_Gear_Mode_Neutral);
			}
			else if (APAstatus == APAStatus::Reversing) // reversing towards the parking space
			{
				if //(UtilMath::CloseEnough(std::atan2(targetParkingSpace.heading.y, targetParkingSpace.heading.x),
					//(double)gpsPtr->oriZ, UtilUnit::DegreeToRad(headingErrorThresholdDegree)))
					(pGps->posY < -14.2)
				{
					APAstatus = APAStatus::Finishing;
					std::cout << "Start reversing straightly!" << std::endl;
				}
				else
				{
					double speedKmH = UtilUnit::MsToKmH(UtilMath::Length({ gpsPtr->velX, gpsPtr->velY, gpsPtr->velZ }));
					double throttle, brake;
					if (speedKmH < 3 && !reversingStarted)
					{
						throttle = 0.03f;
						brake = 0.f;
					}
					else
					{
						reversingStarted = true;
						throttle = 0.f;
						brake = 0.02f;
					}

					double steering = UtilDriver::CalculateSteering(planner.ReverseTrajectory(), gpsPtr.get());
					UtilDriver::SetControl(gpsPtr->timestamp, throttle, brake, steering / 2, ESimOne_Gear_Mode_Reverse);
				}
			}
			else if (APAstatus == APAStatus::Finishing) // finishing
			{
				//double endDistance = UtilMath::Distance({ gpsPtr->posX, gpsPtr->posY }, planner.ParkingEndPoint());
				//if (endDistance < parkingEndDistanceThreshold)
				//{
				std::cout << "Finished parking!" << std::endl;
				UtilDriver::SetControl(gpsPtr->timestamp, 0.1, 0., -1., ESimOne_Gear_Mode_Reverse);
				if (abs(yaw) < 0.05) {
					APAstatus = APAStatus::Waiting;
				}

				//}
				// let the vehicle slide
				//UtilDriver::SetControl(gpsPtr->timestamp, 0., 0., 0., ESimOne_Gear_Mode_Reverse);
			}
			else if (APAstatus == APAStatus::Waiting)
			{
				if (!waitingStarted)
				{
					startTime = std::chrono::system_clock::now();
					waitingStarted = true;
				}
				else
				{
					// wait for a total of waitingTimeSecond
					std::chrono::duration<double> currentWaitingTimeSecond = std::chrono::system_clock::now() - startTime;
					if (currentWaitingTimeSecond.count() > waitingTimeSecond)
					{
						APAstatus = APAStatus::Leaving;
						std::cout << "Finished waiting for " << waitingTimeSecond << " second!" << std::endl;
					}
				}
				UtilDriver::SetControl(gpsPtr->timestamp, 0., 1., 0., ESimOne_Gear_Mode_Neutral);
			}
			else if (APAstatus == APAStatus::Leaving)
			{
				double steering = UtilDriver::CalculateSteering(planner.LeavingTrajectory(), gpsPtr.get());
				if (yaw > 0.8) {
					APAstatus = APAStatus::End;
				}
				UtilDriver::SetControl(gpsPtr->timestamp, 0.05, 0., -1);
			}
			else if (APAstatus == APAStatus::End)
			{
				if (yaw > 0.05)
					UtilDriver::SetControl(gpsPtr->timestamp, 0.2, 0., 1);
				UtilDriver::SetControl(gpsPtr->timestamp, 1, 0, -0.002);
			}
		}
		
		SimOneAPI::NextFrame(frame);
	}

	return 0;
}
