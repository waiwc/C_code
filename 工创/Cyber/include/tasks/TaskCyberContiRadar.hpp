#ifndef TASKCYBERCONTIRADAR_HPP_
#define TASKCYBERCONTIRADAR_HPP_

#include <iostream>
#include <memory>
#include <math.h>
#include <string.h>

#include "CyberWriterContiRadar.hpp"
#include "SimOneSensorAPI.h"

class CW_ContiRadar
{
public:
    CW_ContiRadar(std::string channel_name);
    ~CW_ContiRadar();
    bool set_conti_radar(int& obj_size, CyberWriterContiRadar::Object *obj_list);
    static void set_conti_radar_cb(const char* mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection *pDetections);
    void task_conti_radar();

private:
    CyberWriterContiRadar mCyberContiRadar;
    static CyberWriterContiRadar *mpCyberContiRadar;
};

#endif