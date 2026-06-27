#ifndef TASKCYBERTRAFFICLIGHT_HPP_
#define TASKCYBERTRAFFICLIGHT_HPP_

#include <iostream>
#include <string>
#include <string.h>
#include <vector>
#include "CyberWriterTrafficLight.hpp"
#include "SimOneSensorAPI.h"
#include "SimOneHDMapAPI.h"

#define ID_MAX_LENTH 64
#define LIGHT_MAX_NUM 64

typedef struct
{
    char id[LIGHT_MAX_NUM][ID_MAX_LENTH];
    float confidence[LIGHT_MAX_NUM];
    int color[LIGHT_MAX_NUM];
} TrafficLight_T;

class CW_TrafficLight
{
    public:
        CW_TrafficLight(std::string channel_name);
        ~CW_TrafficLight();

        bool set_traffic_light(TrafficLight_T& data, int& size);
        void task_traffic_light();
    private:
	    CyberWriterTrafficLight mCyberTrafficLight;
};

#endif
