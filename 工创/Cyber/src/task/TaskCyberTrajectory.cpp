#include "TaskCyberTrajectory.hpp"

CW_Trajectory::CW_Trajectory(std::string channel_name):mCyberTrajectory(channel_name.c_str()){}
CW_Trajectory::~CW_Trajectory(){}

// void CW_Trajectory::set_trajectory(std::unique_ptr<SimOne_Data_Control_Trajectory>& pData)
// {
// 	mCyberTrajectory.spinOnce();
// 
//     pData->point_num = mCyberTrajectory.mTrajectorySize;
//     std::cout << "--------------------------- point_num: " << pData->point_num << "\n";
//     for(int i = 0; i < pData->point_num; i++)
//     {
//         pData->points[i].posx = mCyberTrajectory.mTrajectoryList[i].x;
//         pData->points[i].posy = mCyberTrajectory.mTrajectoryList[i].y;
//         pData->points[i].speed = mCyberTrajectory.mTrajectoryList[i].v;
//         std::cout << "pos[" << i << "]: " << pData->points[i].posx << ", " << pData->points[i].posy << ")\n";
//         std::cout << "speed[" << i << "]: " << pData->points[i].speed << "\n";
//     }
// }

void CW_Trajectory::set_trajectory(std::unique_ptr<SimOne_Data_Trajectory> &pData)
{
    mCyberTrajectory.spinOnce();

    pData->trajectorySize = mCyberTrajectory.mTrajectorySize > SOSM_TRAJECTORY_SIZE_MAX ? SOSM_TRAJECTORY_SIZE_MAX : mCyberTrajectory.mTrajectorySize;
    // std::cout << "------------ mTrajectorySize: " << pData->trajectorySize << std::endl;
    for (int i = 0; i < pData->trajectorySize; i++)
    {
        pData->trajectory[i].posX = mCyberTrajectory.mTrajectoryList[i].x;
        pData->trajectory[i].posY = mCyberTrajectory.mTrajectoryList[i].y;
        pData->trajectory[i].vel = mCyberTrajectory.mTrajectoryList[i].v;

        // std::cout << "pos[" << i << "]: " << pData->trajectory[i].posX << ", " << pData->trajectory[i].posY << ")\n";
        // std::cout << "vel[" << i << "]: " << pData->trajectory[i].vel << ")\n";
    }
}

// void CW_Trajectory::set_trajectory(std::unique_ptr<SimOne_Data_Trajectory> &pData)
// {
//     mCyberTrajectory.spinOnce();
// 
//     // Only send 10 trajectory points.
//     int intervalCount = mCyberTrajectory.mTrajectorySize;
//     float interval = 1.0;
// 
//     if (intervalCount > 30)
//     {
//         intervalCount = 30;
//         interval = (mCyberTrajectory.mTrajectorySize / 10.0);
//     }
// 
//     std::cout << "-------------------------------- intervalCount: " << intervalCount << std::endl;
//     for (int i = 0; i < intervalCount; i++)
//     {
//         int n = (int)(i * interval);
//         pData->trajectory[i].posX = mCyberTrajectory.mTrajectoryList[n].x;
//         pData->trajectory[i].posY = mCyberTrajectory.mTrajectoryList[n].y;
//         pData->trajectory[i].vel = mCyberTrajectory.mTrajectoryList[n].v;
//         std::cout << "------ n: " << n << std::endl;
//         std::cout << "#" << i << "  posX = " << pData->trajectory[i].posX << std::endl;
//         std::cout << "#" << i << "  posY = " << pData->trajectory[i].posY << std::endl;
//         std::cout << "#" << i << "  vel = " << pData->trajectory[i].vel << std::endl;
//     }
// }

void CW_Trajectory::task_trajectory()
{
    // std::unique_ptr<SimOne_Data_Control_Trajectory> pControlTrajectory = std::make_unique<SimOne_Data_Control_Trajectory>();
    // set_trajectory(pControlTrajectory);
    // if (!SimOneAPI::SetDriveTrajectory("0", pControlTrajectory.get()));
    // {
    //     std::cout << "SimOneAPI::SetDriveTrajectory Failed!" << std::endl;
    // }

    std::unique_ptr<SimOne_Data_Trajectory> pTrajectory = std::make_unique<SimOne_Data_Trajectory>();
    set_trajectory(pTrajectory);

    if (!SimOneAPI::SetTrajectory("0", pTrajectory.get()))
    {
        std::cout << "SimOneAPI::SetTrajectory Failed!" << std::endl;
    }
}
