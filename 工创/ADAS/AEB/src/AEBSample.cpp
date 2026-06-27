#include "SimOneServiceAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneHDMapAPI.h"
#include "SimOneSensorAPI.h"
#include "SSD/SimPoint3D.h"
#include "UtilMath.h"
#include "hdmap/SampleGetNearMostLane.h"
#include "hdmap/SampleGetLaneST.h"
#include <memory>
#include <limits>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <string>
#include <unordered_map>
#include "SimOneEvaluationAPI.h"

//-----------------------------------------------------------------------------
// 常量定义
//-----------------------------------------------------------------------------
const double A_MAX = 9.7;                  // 最大减速度 (m/s²)
const double V_P_DEFAULT = 0.01;           // 行人默认横穿速度 (m/s)
const double REACTION_TIME = 0.0;          // 反应时间 (s)
const double FIND_PEOPLE_DIS = 0;          // 发现人的距离 (m)
const double HIGH_SPEED = (80.0 / 3.6);    // 最大速度(m/s)
const double SPEED_HYSTERESIS = 0.5f;      // 速度阈值
const double VEHICLE_FRONT_TO_GPS = 3.918; // 车头到后轴（GPS）长度
// TTC分级阈值
const double TTC_LEVEL_WARN = 2.6;
const double TTC_LEVEL_PARTIAL = 1.6;
const double TTC_LEVEL_FULL = 0.6;
// 制动强度
const float BRAKE_LEVEL_WARN = 0.0f;
const float BRAKE_LEVEL_PARTIAL = 0.8f;
const float BRAKE_LEVEL_FULL = 1.0f;
// 静止物体刹车距离参数
const double D_STOP = 0.4;  // 目标停车距离
const double D_BUFFER = 0.0; // 缓冲距离
const double D_FULL = 0.4;   // Full 制动触发距离
const int FRAME_PER_SEC = 60;
//-----------------------------------------------------------------------------
// 枚举定义
//-----------------------------------------------------------------------------
enum class BrakeLevel
{
	None,
	Warning,
	Partial,
	Full
};
//-----------------------------------------------------------------------------
// 障碍物状态结构体
//-----------------------------------------------------------------------------
struct ObstacleState
{
	double lastVelocity = 0.0;
	bool isFirstCall = true;
};

// 假设障碍物有唯一标识符（obs.id），使用 std::unordered_map 保存状态
std::unordered_map<std::string, ObstacleState> obstacleStates;

//-----------------------------------------------------------------------------
// 辅助函数声明
//-----------------------------------------------------------------------------
double CalculateSafeDistance(double speed);
double GetLaneHeading(const SSD::SimString &laneId, double s = 0.0, double t = 0.0);
void SetBrakeLevel(BrakeLevel level, SimOne_Data_Control *control);
BrakeLevel DetermineBrakeLevel(double ttc);
double CalculateTTC(double distance, double relativeSpeed, double relativeAcceleration);

//-----------------------------------------------------------------------------
// 障碍物处理函数声明
//-----------------------------------------------------------------------------
void ProcessObstacle(const SimOne_Data_Obstacle_Entry &obs,
	const SSD::SimPoint3D &vehiclePos, double vehicleSpeed,
	double ownAcceleration, BrakeLevel &bestBrakeLevel,
	double &minCarDistance, double &frontCarSpeed, double &frontCarAccel, int frame);
void ProcessVehicle(const SimOne_Data_Obstacle_Entry &obs,
	double distance, double obsSpeed, double obsAcceleration,
	double vehicleSpeed, double ownAcceleration, BrakeLevel &bestBrakeLevel);
void ProcessPedestrian(const SimOne_Data_Obstacle_Entry &obs,
	const SSD::SimPoint3D &obsPos, double obsSpeed,
	const SSD::SimPoint3D &vehiclePos, double vehicleSpeed,
	BrakeLevel &bestBrakeLevel);
void ProcessStaticObstacle(double distance, double obsSpeed, double vehicleSpeed,
	double ownAcceleration, BrakeLevel &bestBrakeLevel);
//-----------------------------------------------------------------------------
// 主函数
//-----------------------------------------------------------------------------
int main()
{
	// 初始化核心变量
	const char *MAIN_VEHICLE_ID = "0";
	bool switchOnHighSpeed = true;
	bool isSimOneInitialized = false;
	bool isJoinTimeLoop = true;
	bool keepHighSpeed = false;
	bool AEBState = false;
	// 初始化SimOne API
	SimOneAPI::InitSimOneAPI(MAIN_VEHICLE_ID, isJoinTimeLoop);
	SimOneAPI::SetDriverName(MAIN_VEHICLE_ID, "AEB");
	SimOneAPI::SetDriveMode(MAIN_VEHICLE_ID, ESimOne_Drive_Mode_API);
	SimOneAPI::InitEvaluationServiceWithLocalData(MAIN_VEHICLE_ID); // 函数评价初始化
	// 控制指令缓存
	auto control = std::make_unique<SimOne_Data_Control>();
	// 地图加载循环
	int timeout = 20;
	while (true)
	{
		if (SimOneAPI::LoadHDMap(timeout))
		{
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type_Information, "HDMap Information Loaded");
			break;
		}
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type_Warning, "HDMap Loading... Retrying in 5s");
	}
	// 预分配数据对象
	auto gps = std::make_unique<SimOne_Data_Gps>();
	auto obstacle = std::make_unique<SimOne_Data_Obstacle>();
	// 主仿真循环
	while (true)
	{
		int frame = SimOneAPI::Wait();
		ESimOne_Case_Status runStatus = SimOneAPI::GetCaseRunStatus();
		// 根据驻车停止状态进入评价函数
		if (runStatus == ESimOne_Case_Status_Stop)
		{
			SimOneAPI::SaveEvaluationRecord();
			break;
		}

		// 获取车辆状态数据
		if (!SimOneAPI::GetGps(MAIN_VEHICLE_ID, gps.get()))
		{
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type_Warning, "GPS Data Fetch Failed");
		}
		if (!SimOneAPI::GetGroundTruth(MAIN_VEHICLE_ID, obstacle.get()))
		{
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type_Warning, "Obstacle Data Fetch Failed");
		}
		SimOneAPI::GetDriverControl(MAIN_VEHICLE_ID, control.get());

		// 运行时逻辑处理
		if (runStatus == ESimOne_Case_Status_Running)
		{
			if (!isSimOneInitialized)
			{
				SimOneAPI::SetLogOut(ESimOne_LogLevel_Type_Information, "SimOne Initialized!");
				isSimOneInitialized = true;
			}
			// 获取主车状态
			SSD::SimPoint3D vehiclePos(gps->posX, gps->posY, gps->posZ);
			double vehicleSpeed = UtilMath::calculateSpeed(gps->velX, gps->velY, gps->velZ);
			double vehicleS = 0.0;
			double vehicleT = 0.0;
			SimOneAPI::GetLaneST(SampleGetNearMostLane(vehiclePos), vehiclePos, vehicleS, vehicleT);
			// 获取主车道ID
			SSD::SimString mainLaneId = SampleGetNearMostLane(vehiclePos);
			AEBState = false;
			// 通用障碍物处理
			double minObstacleDistance = std::numeric_limits<double>::max();
			bool obstacleDetected = false;
			BrakeLevel bestBrakeLevel = BrakeLevel::None;
			// 前车跟踪变量
			double minCarDistance = std::numeric_limits<double>::max();
			double frontCarSpeed = 0.0;
			double frontCarAccel = 0.0;
			// 单次遍历处理所有障碍物
			for (size_t i = 0; i < obstacle->obstacleSize; ++i)
			{
				ProcessObstacle(obstacle->obstacle[i], vehiclePos, vehicleSpeed,
					gps->accelX, bestBrakeLevel, minCarDistance,
					frontCarSpeed, frontCarAccel, frame);
			}
			// 应用最佳制动级别
			if (bestBrakeLevel != BrakeLevel::None)
			{
				SetBrakeLevel(bestBrakeLevel, control.get());
				AEBState = true;
				// 记录日志
				switch (bestBrakeLevel)
				{
				case BrakeLevel::None:
					SimOneAPI::SetLogOut(ESimOne_LogLevel_Type_Information, "TTC Level 1: No Action");
					break;
				case BrakeLevel::Warning:
					SimOneAPI::SetLogOut(ESimOne_LogLevel_Type_Warning, "TTC Level 2: Warning");
					break;
				case BrakeLevel::Partial:
					SimOneAPI::SetLogOut(ESimOne_LogLevel_Type_Information, "TTC Level 3: 40% Brake");
					break;
				case BrakeLevel::Full:
					SimOneAPI::SetLogOut(ESimOne_LogLevel_Type_Information, "TTC Level 4: Emergency Brake");
					break;
				}
			}
			// 速度保持控制逻辑
			if (!AEBState)
			{
				keepHighSpeed = switchOnHighSpeed;
				if (keepHighSpeed)
				{
					if (vehicleSpeed < HIGH_SPEED - SPEED_HYSTERESIS)
					{
						control->throttle = 1.0f;
						control->brake = 0.0f;
					}
					else if (vehicleSpeed > HIGH_SPEED + SPEED_HYSTERESIS)
					{
						control->throttle = 0.0f;
						control->brake = 0.8f;
					}
					else
					{
						control->throttle = 0.0f;
						control->brake = 0.0f;
					}
				}
			}
			else
			{
				keepHighSpeed = false;
			}
		}
		else
		{
			SimOneAPI::SetLogOut(ESimOne_LogLevel_Type_Information, "SimOne Initializing...");
		}
		// 执行控制指令
		SimOneAPI::SetDrive(MAIN_VEHICLE_ID, control.get());

		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type_Debug, "--------------------------------------------");
		SimOneAPI::NextFrame(frame);
	}
	return 0;
}
//-----------------------------------------------------------------------------
// 辅助函数实现
//-----------------------------------------------------------------------------
double CalculateSafeDistance(double speed)
{
	double brakingDistance = (speed * speed) / (2 * A_MAX);
	return speed * REACTION_TIME + brakingDistance;
}

double GetLaneHeading(const SSD::SimString &laneId, double s, double t)
{
	SimPoint3D inertial, direction;
	bool success = SimOneAPI::GetInertialFromLaneST(laneId, s, t, inertial, direction);
	if (!success)
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type_Warning, "Failed to get lane heading for lane:%c ", laneId);
		return 0.0;
	}
	return std::atan2(direction.y, direction.x); // 返回弧度制航向角
}

void SetBrakeLevel(BrakeLevel level, SimOne_Data_Control *control)
{
	switch (level)
	{
	case BrakeLevel::None:
		control->brake = BRAKE_LEVEL_WARN;
		control->throttle = 1.0f; // 保持油门
		break;
	case BrakeLevel::Warning:
		control->brake = BRAKE_LEVEL_WARN;
		control->throttle = 0.0f;
		break;
	case BrakeLevel::Partial:
		control->brake = BRAKE_LEVEL_PARTIAL;
		control->throttle = 0.0f;
		break;
	case BrakeLevel::Full:
		control->brake = BRAKE_LEVEL_FULL;
		control->throttle = 0.0f;
		break;
	}
}

BrakeLevel DetermineBrakeLevel(double ttc)
{
	if (ttc > TTC_LEVEL_WARN)
	{
		return BrakeLevel::None;
	}
	else if (ttc > TTC_LEVEL_PARTIAL)
	{
		return BrakeLevel::Warning;
	}
	else if (ttc > TTC_LEVEL_FULL)
	{
		return BrakeLevel::Partial;
	}
	else
	{
		return BrakeLevel::Full;
	}
}

double CalculateTTC(double distance, double relativeSpeed, double relativeAcceleration)
{
	if (distance <= 0 || relativeSpeed <= 0)
	{
		return std::numeric_limits<double>::infinity();
	}
	if (fabs(relativeAcceleration) > 1e-6)
	{
		// 使用考虑加速度的TTC公式：t = [sqrt(v² + 2*a*d) - v]/a
		double discriminant = pow(relativeSpeed, 2) + 2 * relativeAcceleration * distance;
		if (discriminant < 0)
		{
			return std::numeric_limits<double>::infinity();
		}
		return (sqrt(discriminant) - relativeSpeed) / relativeAcceleration;
	}
	else
	{
		// 加速度为零时使用基础公式
		return distance / (relativeSpeed + 1e-6); // 防除零
	}
}

//-----------------------------------------------------------------------------
// 障碍物处理函数实现
//-----------------------------------------------------------------------------

void ProcessObstacle(const SimOne_Data_Obstacle_Entry &obs,
	const SSD::SimPoint3D &vehiclePos, double vehicleSpeed,
	double ownAcceleration, BrakeLevel &bestBrakeLevel,
	double &minCarDistance, double &frontCarSpeed, double &frontCarAccel, int frame)
{
	SSD::SimPoint3D obsPos(obs.posX, obs.posY, obs.posZ);
	double obsSpeed = UtilMath::calculateSpeed(obs.velX, obs.velY, obs.velZ);

	// 获取障碍物唯一标识符（假设 obs.id 是唯一标识符）
	std::string obsId = std::to_string(obs.id);

	// 获取或初始化状态
	auto &state = obstacleStates[obsId];

	double obsAcceleration = 0.0;

	double currentVelocity = obs.velX;
	if (!state.isFirstCall)
	{
		double deltaTime = 1.0 / double(FRAME_PER_SEC); // 根据实际帧率调整
		obsAcceleration = (currentVelocity - state.lastVelocity) / deltaTime;
	}
	else
	{
		obsAcceleration = 0.0;
		state.isFirstCall = false;
	}
	state.lastVelocity = currentVelocity;

	double distance = UtilMath::planarDistance(vehiclePos, obsPos) - VEHICLE_FRONT_TO_GPS - obs.length / 2;
	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type_Warning, "distance:%f ", distance);

	// 公共变量更新
	if (distance < minCarDistance && obs.type == ESimOne_Obstacle_Type_Car)
	{
		minCarDistance = distance;
		frontCarSpeed = obsSpeed;
		frontCarAccel = obsAcceleration;
	}

	// 类型分支处理
	switch (obs.type)
	{
	case ESimOne_Obstacle_Type_Car:
		ProcessVehicle(obs, distance, obsSpeed, obsAcceleration, vehicleSpeed, ownAcceleration, bestBrakeLevel);
		break;
	case ESimOne_Obstacle_Type_Pedestrian:
		ProcessPedestrian(obs, obsPos, obsSpeed, vehiclePos, vehicleSpeed, bestBrakeLevel);
		break;
	default:
		ProcessStaticObstacle(distance, obsSpeed, vehicleSpeed, ownAcceleration, bestBrakeLevel);
	}
}

void ProcessPedestrian(const SimOne_Data_Obstacle_Entry &obs,
	const SSD::SimPoint3D &obsPos, double obsSpeed,
	const SSD::SimPoint3D &vehiclePos, double vehicleSpeed,
	BrakeLevel &bestBrakeLevel)
{
	double sPed, tPed = 0.0;
	SSD::SimString mainLaneId = SampleGetNearMostLane(vehiclePos);

	if (SimOneAPI::GetLaneST(mainLaneId, obsPos, sPed, tPed))
	{
		double vehicleS = 0.0;
		double vehicleT = 0.0;
		SimOneAPI::GetLaneST(SampleGetNearMostLane(vehiclePos), vehiclePos, vehicleS, vehicleT);
		double distance = sPed - vehicleS - VEHICLE_FRONT_TO_GPS;
		double laneHeading = GetLaneHeading(mainLaneId);

		// 行人速度分解
		double v_p = std::max(obsSpeed, V_P_DEFAULT);
		double pedHeading = (obs.velX != 0 || obs.velY != 0)
			? std::atan2(obs.velY, obs.velX)
			: obs.oriZ;
		double headingDiff = pedHeading - laneHeading;
		double pedestrianLongitudinalSpeed = v_p * std::cos(headingDiff);

		double relativeSpeed = vehicleSpeed - pedestrianLongitudinalSpeed;

		if (distance > 0.5 && relativeSpeed > 0)
		{
			double ttc = CalculateTTC(distance, relativeSpeed, 0.0);
			BrakeLevel level = DetermineBrakeLevel(ttc);
			// BrakeLevel level = DetermineBrakeLevel(ttc / 1.2);  // 更保守的阈值

			if (level > bestBrakeLevel)
				bestBrakeLevel = level;
		}
	}
}

void ProcessStaticObstacle(double distance, double obsSpeed, double vehicleSpeed,
	double ownAcceleration, BrakeLevel &bestBrakeLevel)
{
	// 计算制动距离
	double d_brake = (vehicleSpeed * vehicleSpeed) / (2.0 * A_MAX);

	// 计算触发制动的最小距离
	double d_trigger = d_brake + D_STOP + D_BUFFER;

	// 分级制动策略
	BrakeLevel level = BrakeLevel::None;

	if (distance <= D_FULL)
	{
		level = BrakeLevel::Full; // 紧急制动，确保停车
	}
	else if (distance <= d_trigger)
	{
		level = BrakeLevel::Partial; // 部分制动，逐步减速
	}
	else
	{
		level = DetermineBrakeLevel(CalculateTTC(distance, vehicleSpeed, ownAcceleration));
	}
	if (level > bestBrakeLevel)
	{
		bestBrakeLevel = level;
	}
}
void ProcessVehicle(const SimOne_Data_Obstacle_Entry &obs,
	double distance, double obsSpeed, double obsAcceleration,
	double vehicleSpeed, double ownAcceleration, BrakeLevel &bestBrakeLevel)
{
	double relativeSpeed = vehicleSpeed - obsSpeed;
	double relativeAcceleration = ownAcceleration - obsAcceleration;
	if (relativeSpeed > 0 || distance < 20.0)
	{
		if (obsSpeed < 0.1)
		{
			{ // 前车静止
				ProcessStaticObstacle(distance, obsSpeed, vehicleSpeed, ownAcceleration, bestBrakeLevel);
				return;
			}
		}
		else
		{
			double ttc = CalculateTTC(distance, relativeSpeed, relativeAcceleration);
			BrakeLevel level = DetermineBrakeLevel(ttc);

			// 前车急刹增强逻辑
			if (obsAcceleration < -1.0 && level < BrakeLevel::Full)
			{
				level = static_cast<BrakeLevel>(static_cast<int>(level) + 1);
				if (level > BrakeLevel::Full)
					level = BrakeLevel::Full;
			}

			if (level > bestBrakeLevel)
				bestBrakeLevel = level;
		}
	}
}