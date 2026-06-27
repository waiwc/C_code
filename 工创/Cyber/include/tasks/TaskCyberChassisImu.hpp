#ifndef TASKCYBERCHASSISIMU_HPP_
#define TASKCYBERCHASSISIMU_HPP_

#include <iostream>
#include <math.h>

#include "CyberWriterChassis.hpp"
#include "CyberWriterImu.hpp"
#include "SimOneSensorAPI.h"

typedef struct
{
    float speedMps;
    float engineRpm;
    int gear;
    float throttleP;
    float steeringP;
    float brakeP;
    int drivingMode;
} Chassis_T;

typedef struct
{
    float angVelX;
    float angVelY;
    float angVelZ;
    float linAccX;
    float linAccY;
    float linAccZ;
} Imu_T;

class CW_Chassis_Imu
{
    public:
        CW_Chassis_Imu(std::string channel_name_chassis, std::string channel_name_imu);
        ~CW_Chassis_Imu();
        bool set_chassis_imu(Chassis_T& data_chassis, Imu_T& data_imu);
        static void set_chassis_imu_cb(const char* mainVehicleId, SimOne_Data_Gps *pGps);
        void task_chassis_imu();
    private:
        CyberWriterChassis mCyberChassis;
        CyberWriterImu mCyberImu;
        static CyberWriterChassis *mpCyberChassis;
        static CyberWriterImu  *mpCyberImu;
};

#endif
