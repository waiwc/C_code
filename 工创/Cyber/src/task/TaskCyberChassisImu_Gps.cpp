#include "TaskCyberChassisImu_Gps.hpp"

CyberWriterChassis *CW_ChassisImuGps::mpCyberChassis = nullptr;
CyberWriterImu  *CW_ChassisImuGps::mpCyberImu = nullptr;
CyberWriterGps *CW_ChassisImuGps::mpCyberGps = nullptr;
CyberWriterInsStat *CW_ChassisImuGps::mpCyberInsStat = nullptr;

CW_ChassisImuGps::CW_ChassisImuGps(std::string channel_name_chassis, std::string channel_name_imu,
    std::string channel_name_gps, std::string channel_name_stat):
    mCyberChassis(channel_name_chassis.c_str()), mCyberImu(channel_name_imu.c_str()),
    mCyberGps(channel_name_gps.c_str()), mCyberInsStat(channel_name_stat.c_str()){}

CW_ChassisImuGps::~CW_ChassisImuGps(){}

void CW_ChassisImuGps::set_chassis_imu_gps_cb(const char *mainVehicleId, SimOne_Data_Gps *pGps)
{
    Chassis_Tcb data_chassis;
    Imu_Tcb data_imu;
    Gps_Tcb data_gps;

    data_chassis.speedMps = sqrt((pGps->velX * pGps->velX) + (pGps->velY * pGps->velY));;
    data_chassis.engineRpm = pGps->engineRpm;
    data_chassis.gear = pGps->gear;
    data_chassis.throttleP = pGps->throttle;
    data_chassis.steeringP = pGps->steering;
    data_chassis.brakeP = pGps->brake;
    // data_chassis.drivingMode = ;

    data_imu.angVelX = pGps->angVelX;
    data_imu.angVelY = pGps->angVelY;
    data_imu.angVelZ = pGps->angVelZ;
    data_imu.linAccX = pGps->accelX;
    data_imu.linAccY = pGps->accelY;
    data_imu.linAccZ = pGps->accelZ;

    cybertron::quat q(cybertron::vec3(pGps->oriX, pGps->oriY, pGps->oriZ - (M_PI/ 2)));
    data_gps.posX = pGps->posX;
    data_gps.posY = pGps->posY;
    data_gps.posZ = pGps->posZ;
    data_gps.mVelocityX = pGps->velX;
    data_gps.mVelocityY = pGps->velY;
    data_gps.mVelocityZ = pGps->velZ;
    data_gps.oriX = q.x;
    data_gps.oriY = q.y;
    data_gps.oriZ = q.z;
    data_gps.oriW = q.w;

    mpCyberChassis->publish(data_chassis.speedMps, data_chassis.engineRpm, data_chassis.gear, data_chassis.throttleP, data_chassis.steeringP, data_chassis.brakeP, 1);    // 1: COMPLETE_AUTO_DRIVE
    mpCyberImu->publish(data_imu.angVelX, data_imu.angVelY, data_imu.angVelZ, data_imu.linAccX, data_imu.linAccY, data_imu.linAccZ);

    mpCyberGps->publish(data_gps.posX, data_gps.posY, data_gps.posZ, data_gps.mVelocityX, data_gps.mVelocityY, data_gps.mVelocityZ, data_gps.oriX, data_gps.oriY, data_gps.oriZ, data_gps.oriW);
    mpCyberInsStat->publish();
}

void CW_ChassisImuGps::task_chassis_imu_gps()
{
    mpCyberChassis = &mCyberChassis;
    mpCyberImu = &mCyberImu;
    mpCyberGps = &mCyberGps;
    mpCyberInsStat = &mCyberInsStat;

    if (!SimOneAPI::SetGpsUpdateCB(set_chassis_imu_gps_cb))
    {
        printf("\033[1m\033[31m[task_gps]: SetSimOneGpsCB Failed!\033[0m\n");
    }
}
