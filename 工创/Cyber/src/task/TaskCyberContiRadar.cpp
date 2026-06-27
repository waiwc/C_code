#include "TaskCyberContiRadar.hpp"

CyberWriterContiRadar *CW_ContiRadar::mpCyberContiRadar = nullptr;

CW_ContiRadar::CW_ContiRadar(std::string channel_name):mCyberContiRadar(channel_name.c_str()){}
CW_ContiRadar::~CW_ContiRadar(){}

bool CW_ContiRadar::set_conti_radar(int& obj_size, CyberWriterContiRadar::Object *obj_list)
{
    static int lastFrame = 0;
    std::unique_ptr<SimOne_Data_RadarDetection> pDetections = std::make_unique<SimOne_Data_RadarDetection>();

    if (!SimOneAPI::GetRadarDetections("0", "objectBasedRadar1", pDetections.get()))
    {
        std::cout << "[CW_ContiRadar] GetRadarDetections Failed!" << std::endl;
        return false;
    }
    if (pDetections->frame == lastFrame)
    {
        return false;
    }
    lastFrame = pDetections->frame;

    obj_size = pDetections->detectNum;
    for (int i=0; i<obj_size; i++)
    {
        obj_list[i].id = pDetections->detections[i].id;
        obj_list[i].longitudeDist = pDetections->detections[i].range * sin(pDetections->detections[i].azimuth);
        obj_list[i].lateralDist = pDetections->detections[i].range * cos(pDetections->detections[i].azimuth);
        obj_list[i].longitudeVel = pDetections->detections[i].velX;
        obj_list[i].lateralVel = pDetections->detections[i].velY;
        obj_list[i].rcs = pDetections->detections[i].snrdb;
        obj_list[i].dynprop = 4;
    }

    return true;
}

// void CW_ContiRadar::task_conti_radar()
// {
//     mCyberContiRadar.mObjectSize = 0;
//     memset(mCyberContiRadar.mObjectList, 0, sizeof(CyberWriterContiRadar::Object) * OBJECT_MAX_SIZE);
// 
//     if (set_conti_radar(mCyberContiRadar.mObjectSize, mCyberContiRadar.mObjectList))
//     {
//         mCyberContiRadar.publish();
//     }
// }


void CW_ContiRadar::set_conti_radar_cb(const char* mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection *pDetections)
{
    mpCyberContiRadar->mObjectSize = 0;
    memset(mpCyberContiRadar->mObjectList, 0, sizeof(CyberWriterContiRadar::Object) * OBJECT_MAX_SIZE);

    // if (0 == strcmp("objectBasedRadar1", sensorId))

    mpCyberContiRadar->mObjectSize = pDetections->detectNum;
    for (int i=0; i<pDetections->detectNum; i++)
    {
        mpCyberContiRadar->mObjectList[i].id = pDetections->detections[i].id;
        mpCyberContiRadar->mObjectList[i].longitudeDist = pDetections->detections[i].range * sin(pDetections->detections[i].azimuth);
        mpCyberContiRadar->mObjectList[i].lateralDist = pDetections->detections[i].range * cos(pDetections->detections[i].azimuth);
        mpCyberContiRadar->mObjectList[i].longitudeVel = pDetections->detections[i].velX;
        mpCyberContiRadar->mObjectList[i].lateralVel = pDetections->detections[i].velY;
        mpCyberContiRadar->mObjectList[i].rcs = pDetections->detections[i].snrdb;
        mpCyberContiRadar->mObjectList[i].dynprop = 4;
    }

    mpCyberContiRadar->publish();
}

void CW_ContiRadar::task_conti_radar()
{
    mpCyberContiRadar = &mCyberContiRadar;

	if (!SimOneAPI::SetRadarDetectionsUpdateCB(set_conti_radar_cb))
    {
        printf("\033[1m\033[31m[task_conti_radar]: SetRadarDetectionsUpdateCB Failed!\033[0m\n");
    }
}
