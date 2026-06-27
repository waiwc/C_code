#ifndef TASKCYBERGPS_HPP_
#define TASKCYBERGPS_HPP_

#include <iostream>
#include <math.h>

#include "CyberWriterGps.hpp"
#include "CyberWriterInsStat.hpp"
#include "SimOneSensorAPI.h"
#include "glm/quat.hpp"
#include "glm/vec3.hpp"

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
} Gps_T;

class CW_Gps
{
    public:
        CW_Gps(std::string channel_name_gps, std::string channel_name_stat);
        ~CW_Gps();
        bool set_gps(Gps_T& data);
        static void set_gps_cb(const char *mainVehicleId, SimOne_Data_Gps *pGps);
        void task_gps();

    private:
	    CyberWriterGps mCyberGps;
	    CyberWriterInsStat mCyberInsStat;
	    static CyberWriterGps *mpCyberGps;
	    static CyberWriterInsStat *mpCyberInsStat;
};

#endif
