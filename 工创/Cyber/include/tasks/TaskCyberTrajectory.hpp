#ifndef TASKCYBERTRAJECTORY_HPP_
#define TASKCYBERTRAJECTORY_HPP_

#include "CyberReaderTrajectory.hpp"
#include "SimOnePNCAPI.h"
#include <iostream>

class CW_Trajectory
{
    public:
        CW_Trajectory(std::string channel_name);
        ~CW_Trajectory();
        // void set_trajectory(std::unique_ptr<SimOne_Data_Control_Trajectory>& pData);
        void set_trajectory(std::unique_ptr<SimOne_Data_Trajectory> &pData);
        void task_trajectory();

    private:
        CyberReaderTrajectory mCyberTrajectory;
};

#endif
