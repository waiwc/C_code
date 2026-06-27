#ifndef TASKCYBERCHASSISIMUGPS_HPP_
#define TASKCYBERCHASSISIMUGPS_HPP_

#include <iostream>
#include <math.h>

#include "CyberWriterChassis.hpp"
#include "CyberWriterImu.hpp"
#include "CyberWriterGps.hpp"
#include "CyberWriterInsStat.hpp"

#include "SimOneSensorAPI.h"
#include "glm/quat.hpp"
#include "glm/vec3.hpp"

typedef struct
{
    float speedMps;
    float engineRpm;
    int gear;
    float throttleP;
    float steeringP;
    float brakeP;
    int drivingMode;
} Chassis_Tcb;

typedef struct
{
    float angVelX;
    float angVelY;
    float angVelZ;
    float linAccX;
    float linAccY;
    float linAccZ;
} Imu_Tcb;

typedef struct
{
    float posX;
    float posY;
    float posZ;
    float mVelocityX;
    float mVelocityY;
    float mVelocityZ;
    float oriX;
    float oriY;
    float oriZ;
    float oriW;
} Gps_Tcb;

class CW_ChassisImuGps
{
    public:
        CW_ChassisImuGps(std::string channel_name_chassis, std::string channel_name_imu, std::string channel_name_gps, std::string channel_name_stat);
        ~CW_ChassisImuGps();
        static void set_chassis_imu_gps_cb(const char *mainVehicleId, SimOne_Data_Gps *pGps);
        void task_chassis_imu_gps();

    private:
        CyberWriterChassis mCyberChassis;
        CyberWriterImu mCyberImu;
	    CyberWriterGps mCyberGps;
	    CyberWriterInsStat mCyberInsStat;
        static CyberWriterChassis *mpCyberChassis;
        static CyberWriterImu  *mpCyberImu;
	    static CyberWriterGps *mpCyberGps;
	    static CyberWriterInsStat *mpCyberInsStat;
};

#endif
