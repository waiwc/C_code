#ifndef TASKCYBEROBSTACLES_HPP_
#define TASKCYBEROBSTACLES_HPP_

#include <iostream>
#include <string.h>

#include "CyberWriterObstacles.hpp"
#include "SimOneSensorAPI.h"
#include <atomic>

class CW_Obstacles
{
    public:
        CW_Obstacles(std::string channel_name);
        ~CW_Obstacles();

        void set_null_obstacles();
        bool set_obstacles(int& obj_size, CyberWriterObstacles::Obstacle* obj_list);
        static void set_obstacles_cb(const char* mainVehicleId, SimOne_Data_Obstacle *pObstacle);
        void task_obstacles();

    private:
	    CyberWriterObstacles mCyberObstacles;
	    static CyberWriterObstacles *mpCyberObstacles;
        static uint64_t  frame;
        static uint64_t  frame_counter;
};

#endif
