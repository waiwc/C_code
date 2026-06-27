#include "SimOneServiceAPI.h"
#include "SimOneSensorAPI.h"
#include "SimOneHDMapAPI.h"
#include "SSD/SimPoint3D.h"
#include "UtilDriver.h"
#include "UtilMath.h"
#include "hdmap\SampleGetNearMostLane.h"
#include "hdmap\SampleGetLaneST.h"
#include "SimOneEvaluationAPI.h"
#include <iostream>
#include <memory>
#include <cstring>

//-----------------------------------------------------------------------------
// 常量定义
//-----------------------------------------------------------------------------
#define ON (1)
#define OFF (0)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MINMAX(input, low, upper) MIN(MAX(input, low), upper)

uint8_t left_lane = OFF;               // 左侧车道是否存在
uint8_t right_lane = OFF;              // 右侧车道是否存在
uint8_t change_road = OFF;             // 是否变道
uint8_t use_fixd_steering = OFF;       // 是否使用固定转向
uint16_t dis_CR = 5;                   // 变道默认距离
float defalut_brake = 0.0f;            // 默认刹车值
float defalut_throttle = 0.12f;        // 默认油门值
SSD::SimString mainVehicleLaneId;      // 主车所在车道ID
SSD::SimString mainVehicleLaneId_last; // 上一帧主车所在车道ID
SSD::SimPoint3DVector inputPoints;     // 输入路径点
SSD::SimPoint3DVector targetPath;      // 目标路径
SSD::SimPoint3D targetPointPos;        // 目标位置

//-----------------------------------------------------------------------------
// 辅助函数声明
//-----------------------------------------------------------------------------
void lane_reset(void);                                                                                                                                                     // 车道重置函数
void change_road_function(SSD::SimPoint3DVector *inps, SSD::SimPoint3D &start_p, SSD::SimPoint3D &end_p);                                                                  // 变道函数
bool IsCarInLeftLane(SSD::SimPoint3D MainCarPos, SSD::SimString MainCarLandId, SimOne_Data_Obstacle *obstacle, HDMapStandalone::MLaneLink laneLink, double &LefrCarSpeed); // 检查左侧是否有车辆

bool DetectObstacleInSameLane(const SimOne_Data_Obstacle &obstacleData, const SSD::SimPoint3D &mainCarPos, SSD::SimString mainVehicleLaneId,
                              double &minDistance, int &potentialObstacleIndex, SSD::SimString &potentialObstacleLaneId); // 检测同车道障碍物

void HandleLaneChange(SSD::SimPoint3D mainVehiclePos, SSD::SimPoint3D targetPointPos, uint8_t left_lane, uint8_t right_lane,
                      uint8_t &change_road, uint8_t &use_fixd_steering, float &steering, SSD::SimPoint3DVector &inputPoints, SSD::SimPoint3DVector &targetPath); // 处理车道变更

void UpdateControlForCollisionAvoidance(double minDistance, double mainVehicleSpeed, double obstacleSpeed, ESimOne_Obstacle_Type obstacleType,
                                        SimOne_Data_Control *control, bool AEBState); // 碰撞避免控制更新

void UpdateControlForLaneChange(uint8_t change_road, uint8_t left_lane, uint8_t right_lane, float dis_CR, float minDistance,
                                SimOne_Data_Control *control, SimOne_Data_Signal_Lights *light); // 更新变道控制

void UpdateControlForLeftCar(SSD::SimPoint3D mainCarPos, SSD::SimString mainVehicleLaneId, SimOne_Data_Obstacle *obstacle,
                             HDMapStandalone::MLaneLink laneLink, double mainVehicleSpeed, SimOne_Data_Control *control); // 左侧车辆控制更新
//-----------------------------------------------------------------------------
// 主函数
//-----------------------------------------------------------------------------
int main()
{
    // 初始化核心变量
    const char *MAIN_VEHICLE_ID = "0";
    bool isSimOneInitialized = false;
    bool isJoinTimeLoop = true;
    uint64_t fps_count = 0;
    bool AEBState = false;

    // 初始化SimOne API
    SimOneAPI::InitSimOneAPI(MAIN_VEHICLE_ID, isJoinTimeLoop);
    SimOneAPI::SetDriverName(MAIN_VEHICLE_ID, "OTTOPILOT");
    SimOneAPI::InitEvaluationServiceWithLocalData(MAIN_VEHICLE_ID); // 函数评价初始化

    // 预分配数据对象
    auto gps = std::make_unique<SimOne_Data_Gps>();
    auto obstacle = std::make_unique<SimOne_Data_Obstacle>();

    // 地图加载循环
    int timeout = 20;
    while (true)
    {
        if (SimOneAPI::LoadHDMap(timeout))
        {
            SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loaded");
            break;
        }
        SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loading...");
    }

    // 获取路径点
    std::unique_ptr<SimOne_Data_WayPoints> pWayPoints = std::make_unique<SimOne_Data_WayPoints>();
    if (SimOneAPI::GetWayPoints(MAIN_VEHICLE_ID, pWayPoints.get()))
    {
        for (size_t i = 0; i < pWayPoints->wayPointsSize; ++i)
        {
            SSD::SimPoint3D inputWayPoints(pWayPoints->wayPoints[i].posX, pWayPoints->wayPoints[i].posY, 0);
            inputPoints.push_back(inputWayPoints);
        }
        targetPointPos = inputPoints.back();
    }
    else
    {
        SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Get mainVehicle wayPoints failed");
        return -1;
    }

    // 生成路线
    if (pWayPoints->wayPointsSize >= 2)
    {
        SSD::SimVector<int> indexOfValidPoints;
        if (!SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath))
        {
            SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Generate mainVehicle route failed");
            return -1;
        }
    }
    else if (pWayPoints->wayPointsSize == 1)
    {
        SSD::SimString laneIdInit = SampleGetNearMostLane(inputPoints[0]);
        HDMapStandalone::MLaneInfo laneInfoInit;
        if (!SimOneAPI::GetLaneSample(laneIdInit, laneInfoInit))
        {
            SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Generate mainVehicle initial route failed");
            return -1;
        }
        else
        {
            targetPath = laneInfoInit.centerLine;
        }
    }
    else
    {
        SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Number of wayPoints is zero");
        return -1;
    }

    // 主循环
    while (true)
    {
        fps_count++;
        lane_reset();
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
            SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch GPS failed");
        }

        if (!SimOneAPI::GetGroundTruth(MAIN_VEHICLE_ID, obstacle.get()))
        {
            SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch obstacle failed");
        }

        // 案例运行时处理逻辑
        if (runStatus == ESimOne_Case_Status::ESimOne_Case_Status_Running)
        {
            if (!isSimOneInitialized)
            {
                SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initialized!");
                isSimOneInitialized = true;
            }
            // 获取主车状态
            SSD::SimPoint3D mainVehiclePos(gps->posX, gps->posY, gps->posZ);
            double mainVehicleSpeed = UtilMath::calculateSpeed(gps->velX, gps->velY, gps->velZ);

            double minDistance = std::numeric_limits<double>::max();
            int potentialObstacleIndex = obstacle->obstacleSize;
            mainVehicleLaneId_last = mainVehicleLaneId;
            mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos);
            if (fps_count == 1)
            {
                mainVehicleLaneId_last = mainVehicleLaneId;
            }

            SSD::SimString potentialObstacleLaneId = "";
            if (!DetectObstacleInSameLane(*obstacle, mainVehiclePos, mainVehicleLaneId, minDistance, potentialObstacleIndex, potentialObstacleLaneId))
            {
                // No obstacle found in same lane
            }

            auto &potentialObstacle = obstacle->obstacle[potentialObstacleIndex];
            double obstacleSpeed = UtilMath::calculateSpeed(potentialObstacle.velX, potentialObstacle.velY, potentialObstacle.velZ);
            SSD::SimPoint3D potentialObstaclePos(potentialObstacle.posX, potentialObstacle.posY, potentialObstacle.posZ);
            double sObstalce = 0.;
            double tObstacle = 0.;

            double sMainVehicle = 0.;
            double tMainVehicle = 0.;

            bool isObstalceBehind = false;
            if (!potentialObstacleLaneId.Empty())
            {

                SampleGetLaneST(potentialObstacleLaneId, potentialObstaclePos, sObstalce, tObstacle);
                SampleGetLaneST(mainVehicleLaneId, mainVehiclePos, sMainVehicle, tMainVehicle);

                isObstalceBehind = !(sMainVehicle >= sObstalce);
            }
            // 拿到控制和车灯
            auto control = std::make_unique<SimOne_Data_Control>();
            auto light = std::make_unique<SimOne_Data_Signal_Lights>();

            control->throttle = 0.12f;
            control->brake = defalut_brake;
            control->steering = 0.f;
            control->handbrake = false;
            control->isManualGear = false;
            control->gear = static_cast<ESimOne_Gear_Mode>(1);
            light->signalLights = ESimOne_Signal_Light::ESimOne_Signal_Light_None;

            HDMapStandalone::MLaneLink laneLink;
            SimOneAPI::GetLaneLink(mainVehicleLaneId, laneLink);
            // 判断左右方有无车道
            if (strlen(laneLink.leftNeighborLaneName.GetString()) != 0)
            {
                left_lane = ON;
            }
            if (strlen(laneLink.rightNeighborLaneName.GetString()) != 0)
            {
                right_lane = ON;
            }

            if (isObstalceBehind && change_road == OFF) // 不能变道
            {
                if ((left_lane == ON || right_lane == ON) && minDistance <= 40 && std::abs(obstacleSpeed) <= 2 && fps_count >= 5)
                {
                    if (minDistance <= 20 && fps_count <= 400)
                    {
                        if (fps_count <= 150)
                        {
                            control->gear = static_cast<ESimOne_Gear_Mode>(2);
                            control->brake = 1.0f;
                            SimOneAPI::SetDrive(MAIN_VEHICLE_ID, control.get());
                            std::cout << "waiting\n"
                                      << std::endl;
                        }
                        else
                        {
                            control->gear = static_cast<ESimOne_Gear_Mode>(2);
                            SimOneAPI::SetDrive(MAIN_VEHICLE_ID, control.get());
                            std::cout << "back back\n"
                                      << std::endl;
                        }
                        goto END;
                    }
                    if (minDistance <= 22)
                    {
                        change_road = ON;
                        use_fixd_steering = ON;
                    }
                    else
                    {
                        if ((int(sqrtf(pow(gps->velX, 2) + pow(gps->velY, 2))) * 3.6f) > 10) // 速度大于10km/h
                        {
                            control->throttle = 0.0f;
                            control->brake = 0.3f;
                        }
                        else
                        {
                            control->throttle = 0.2f;
                            control->brake = 0.0f;
                        }
                    }
                }
                else
                {
                    if (obstacle->obstacleSize == 3)
                    {
                        if (minDistance <= 11.5) // 小于11.5就踩死
                        {
                            control->throttle = 0.f;
                            control->brake = 1.0f;
                        }
                    }
                    else
                    {
                        if (minDistance > 11.5 && minDistance <= 40) // 计算ttc（无加速度）
                        {
                            double defaultDistance = 10.;
                            double timeToCollision = std::abs((minDistance - defaultDistance)) / (obstacleSpeed - mainVehicleSpeed);
                            double defautlTimeToCollision = 3.4;
                            if (-timeToCollision < defautlTimeToCollision && timeToCollision < 0)
                            {
                                AEBState = true;
                                control->brake = (float)(mainVehicleSpeed * 3.6 * 0.65 + 0.20);
                            }

                            if (AEBState)
                            {
                                control->throttle = 0.05f;
                            }
                            if (minDistance <= 25 && std::abs(obstacleSpeed) <= 1.0 && potentialObstacle.type == ESimOne_Obstacle_Type::ESimOne_Obstacle_Type_Car)
                            {
                                control->throttle = 0.f;
                                control->brake = 1.0f;
                            }
                        }
                        else if (minDistance > 40 && potentialObstacle.type == ESimOne_Obstacle_Type::ESimOne_Obstacle_Type_Car)
                        {
                            control->throttle = 0.2f;
                        }
                    }
                }
            }
            else if (change_road == ON) // 可以变道
            {
                control->throttle = 0.0f;
                if (left_lane == ON)
                {
                    control->steering = MINMAX(-dis_CR / (float)minDistance, -0.3f, 0.3f);
                    light->signalLights = ESimOne_Signal_Light::ESimOne_Signal_Light_LeftBlinker;
                }
                else if (right_lane == ON)
                {
                    control->steering = MINMAX(dis_CR / (float)minDistance, -0.3f, 0.3f);
                    light->signalLights = ESimOne_Signal_Light::ESimOne_Signal_Light_RightBlinker;
                }

                if (strcmp(mainVehicleLaneId.GetString(), mainVehicleLaneId_last.GetString()) != 0) // 是否换车道了
                {
                    change_road = OFF;
                    use_fixd_steering = OFF;
                    targetPath.clear();
                    change_road_function(&inputPoints, mainVehiclePos, targetPointPos); // 换道
                    SSD::SimVector<int> tempindexOfValidPoints;
                    if (!SimOneAPI::GenerateRoute(inputPoints, tempindexOfValidPoints, targetPath))
                    {
                        SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Generate mainVehicle route failed when finish change road");
                        return -1;
                    }
                }
            }

            double LefrCarSpeed = 0.0;
            if (IsCarInLeftLane(mainVehiclePos, mainVehicleLaneId, obstacle.get(), laneLink, LefrCarSpeed))
            {
                if ((mainVehicleSpeed) > LefrCarSpeed / 2.0)
                {
                    control->throttle = 0.0f;
                    control->brake = 0.3f;
                }
                else
                {
                    control->throttle = 0.2f;
                    control->brake = 0.0f;
                }
            }

            if (use_fixd_steering == OFF)
            {
                double steering = UtilDriver::calculateSteering(targetPath, gps.get());
                control->steering = (float)steering * 1.5f;
            }

            SimOneAPI::SetSignalLights(MAIN_VEHICLE_ID, light.get());
            SimOneAPI::SetDrive(MAIN_VEHICLE_ID, control.get());
        }
        else
        {
            SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initializing...");
        }
    END:
        SimOneAPI::NextFrame(frame);
    }
    return 0;
}

void lane_reset(void)
{
    left_lane = OFF;
    right_lane = OFF;
}

void change_road_function(SSD::SimPoint3DVector *inps, SSD::SimPoint3D &start_p, SSD::SimPoint3D &end_p)
{
    inps->clear();
    inps->push_back(start_p);
    inps->push_back(end_p);
}

bool IsCarInLeftLane(SSD::SimPoint3D MainCarPos, SSD::SimString MainCarLandId, SimOne_Data_Obstacle *obstacle, HDMapStandalone::MLaneLink laneLink, double &LefrCarSpeed)
{
    bool res = false;
    SSD::SimPoint3D CarPos;
    double MinDistance = std::numeric_limits<double>::max();
    SSD::SimString CarLaneId = "";
    double s, t, s_toCenterLine, t_toCenterLine;
    for (size_t i = 0; i < obstacle->obstacleSize; ++i)
    {
        if (obstacle->obstacle[i].type == ESimOne_Obstacle_Type::ESimOne_Obstacle_Type_Car)
        {

            CarPos = {obstacle->obstacle[i].posX, obstacle->obstacle[i].posY, obstacle->obstacle[i].posZ};
            SimOneAPI::GetNearMostLane(CarPos, CarLaneId, s, t, s_toCenterLine, t_toCenterLine);
            double Distance = UtilMath::planarDistance(MainCarPos, CarPos);

            if (strcmp(CarLaneId.GetString(), laneLink.leftNeighborLaneName.GetString()) == 0 && Distance <= 35.0)
            {
                if (Distance < MinDistance)
                {
                    MinDistance = Distance;
                    LefrCarSpeed = UtilMath::calculateSpeed(obstacle->obstacle[i].velX, obstacle->obstacle[i].velY, obstacle->obstacle[i].velZ);
                    res = true;
                }
            }
        }
    }
    return res;
}

bool DetectObstacleInSameLane(const SimOne_Data_Obstacle &obstacleData, const SSD::SimPoint3D &mainCarPos, SSD::SimString mainVehicleLaneId,
                              double &minDistance, int &potentialObstacleIndex, SSD::SimString &potentialObstacleLaneId)
{
    for (size_t i = 0; i < obstacleData.obstacleSize; ++i)
    {
        SSD::SimPoint3D obstaclePos(obstacleData.obstacle[i].posX, obstacleData.obstacle[i].posY, obstacleData.obstacle[i].posZ);
        SSD::SimString obstacleLaneId = SampleGetNearMostLane(obstaclePos);
        if (mainVehicleLaneId == obstacleLaneId)
        {
            double obstacleDistance = UtilMath::planarDistance(mainCarPos, obstaclePos);

            if (obstacleDistance < minDistance)
            {
                minDistance = obstacleDistance;
                potentialObstacleIndex = (int)i;
                potentialObstacleLaneId = obstacleLaneId;
            }
        }
    }
    return potentialObstacleIndex < obstacleData.obstacleSize;
}

void HandleLaneChange(SSD::SimPoint3D mainVehiclePos, SSD::SimPoint3D targetPointPos, uint8_t left_lane, uint8_t right_lane,
                      uint8_t &change_road, uint8_t &use_fixd_steering, float &steering, SSD::SimPoint3DVector &inputPoints, SSD::SimPoint3DVector &targetPath)
{
    if (change_road == ON)
    {
        if (left_lane == ON)
        {
            steering = MINMAX(-dis_CR / (float)targetPath.size(), -0.3f, 0.3f);
        }
        else if (right_lane == ON)
        {
            steering = MINMAX(dis_CR / (float)targetPath.size(), -0.3f, 0.3f);
        }
    }
}

void UpdateControlForCollisionAvoidance(double minDistance, double mainVehicleSpeed, double obstacleSpeed, ESimOne_Obstacle_Type obstacleType,
                                        SimOne_Data_Control *control, bool AEBState)
{
    if (minDistance > 11.5 && minDistance <= 40)
    {
        double defaultDistance = 10.;
        double timeToCollision = std::abs((minDistance - defaultDistance)) / (obstacleSpeed - mainVehicleSpeed);
        double defautlTimeToCollision = 3.4;
        if (-timeToCollision < defautlTimeToCollision && timeToCollision < 0)
        {
            AEBState = true;
            control->brake = (float)(mainVehicleSpeed * 3.6 * 0.65 + 0.20);
        }

        if (AEBState)
        {
            control->throttle = 0.05f;
        }
        if (minDistance <= 25 && std::abs(obstacleSpeed) <= 1.0 && obstacleType == ESimOne_Obstacle_Type::ESimOne_Obstacle_Type_Car)
        {
            control->throttle = 0.f;
            control->brake = 1.0f;
        }
    }
    else if (minDistance > 40 && obstacleType == ESimOne_Obstacle_Type::ESimOne_Obstacle_Type_Car)
    {
        control->throttle = 0.2f;
    }
}

void UpdateControlForLaneChange(uint8_t change_road, uint8_t left_lane, uint8_t right_lane, float dis_CR, float minDistance,
                                SimOne_Data_Control *control, SimOne_Data_Signal_Lights *light)
{
    if (change_road == ON)
    {
        control->throttle = 0.0f;
        if (left_lane == ON)
        {
            control->steering = MINMAX(-dis_CR / (float)minDistance, -0.3f, 0.3f);
            light->signalLights = ESimOne_Signal_Light::ESimOne_Signal_Light_LeftBlinker;
        }
        else if (right_lane == ON)
        {
            control->steering = MINMAX(dis_CR / (float)minDistance, -0.3f, 0.3f);
            light->signalLights = ESimOne_Signal_Light::ESimOne_Signal_Light_RightBlinker;
        }
    }
}

void UpdateControlForLeftCar(SSD::SimPoint3D mainCarPos, SSD::SimString mainVehicleLaneId, SimOne_Data_Obstacle *obstacle,
                             HDMapStandalone::MLaneLink laneLink, double mainVehicleSpeed, SimOne_Data_Control *control)
{
    double LefrCarSpeed = 0.0;
    if (IsCarInLeftLane(mainCarPos, mainVehicleLaneId, obstacle, laneLink, LefrCarSpeed))
    {
        if ((mainVehicleSpeed) > LefrCarSpeed / 2.0)
        {
            control->throttle = 0.0f;
            control->brake = 0.3f;
        }
        else
        {
            control->throttle = 0.2f;
            control->brake = 0.0f;
        }
    }
}