#ifndef TASKCYBERCONTROL_HPP_
#define TASKCYBERCONTROL_HPP_

#include "CyberReaderControl.hpp"
#include "SimOnePNCAPI.h"

#include <iostream>

// typedef struct
// {
//     float mThrottle{0.0};
//     float mSteering{0.0};
//     float mBrake{0.0};
//     int mGear{0};
// } Ctl_T;

class CW_Ctl
{
    public:
        CW_Ctl(std::string channel_name);
        ~CW_Ctl();
        void set_ctl(std::unique_ptr<SimOne_Data_Control>& pData);
        void task_ctl();
    private:
	    CyberReaderControl mCyberControl;
};

#endif