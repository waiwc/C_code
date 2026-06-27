#include "TaskCyberGps.hpp"

CyberWriterGps *CW_Gps::mpCyberGps = nullptr;
CyberWriterInsStat *CW_Gps::mpCyberInsStat = nullptr;

CW_Gps::CW_Gps(std::string channel_name_gps, std::string channel_name_stat):
    mCyberGps(channel_name_gps.c_str()), mCyberInsStat(channel_name_stat.c_str()){}
CW_Gps::~CW_Gps(){}

bool CW_Gps::set_gps(Gps_T& data)
{
    static int lastFrame = 0;
    std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();

    if (!SimOneAPI::GetGps(0, pGps.get()))
    {
        std::cout << "[CW_Gps] GetGps Failed!" << std::endl;
        return false;
    }
    if (pGps->frame == lastFrame)
    {
        return false;
    }
    lastFrame = pGps->frame;

    cybertron::quat q(cybertron::vec3(pGps->oriX, pGps->oriY, pGps->oriZ - (M_PI/ 2)));

    data.posX = pGps->posX;
    data.posY = pGps->posY;
    data.posZ = pGps->posZ;
    data.mVelocityX = pGps->velX;
    data.mVelocityY = pGps->velY;
    data.mVelocityZ = pGps->velZ;
    data.oriX = q.x;
    data.oriY = q.y;
    data.oriZ = q.z;
    data.oriW = q.w;

	// std::cout << "------------ pGps->posX: " << pGps->posX << std::endl;
	// std::cout << "------------ pGps->posY: " << pGps->posY  << std::endl;
	// std::cout << "------------ pGps->posZ: " << pGps->posZ << std::endl;

    return true;
}

// void CW_Gps::task_gps()
// {
//     Gps_T data;
//     if (set_gps(data))
//     {
//         mCyberGps.publish(data.posX, data.posY, data.posZ, data.mVelocityX, data.mVelocityY, data.mVelocityZ, data.oriX, data.oriY, data.oriZ, data.oriW);
//         mCyberInsStat.publish();
//     }
// }

void CW_Gps::set_gps_cb(const char* mainVehicleId, SimOne_Data_Gps *pGps)
{
    Gps_T data;

    cybertron::quat q(cybertron::vec3(pGps->oriX, pGps->oriY, pGps->oriZ - (M_PI/ 2)));
    data.posX = pGps->posX;
    data.posY = pGps->posY;
    data.posZ = pGps->posZ;
    data.mVelocityX = pGps->velX;
    data.mVelocityY = pGps->velY;
    data.mVelocityZ = pGps->velZ;
    data.oriX = q.x;
    data.oriY = q.y;
    data.oriZ = q.z;
    data.oriW = q.w;

    mpCyberGps->publish(data.posX, data.posY, data.posZ, data.mVelocityX, data.mVelocityY, data.mVelocityZ, data.oriX, data.oriY, data.oriZ, data.oriW);
    mpCyberInsStat->publish();
}

void CW_Gps::task_gps()
{
    mpCyberGps = &mCyberGps;
    mpCyberInsStat = &mCyberInsStat;

    if (!SimOneAPI::SetGpsUpdateCB(set_gps_cb))
    {
        printf("\033[1m\033[31m[task_gps]: SetSimOneGpsCB Failed!\033[0m\n");
    }
}
