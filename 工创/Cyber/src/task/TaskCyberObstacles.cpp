#include "TaskCyberObstacles.hpp"

CyberWriterObstacles *CW_Obstacles::mpCyberObstacles = nullptr;
uint64_t  CW_Obstacles::frame = 0;
uint64_t  CW_Obstacles::frame_counter = 0;

CW_Obstacles::CW_Obstacles(std::string channel_name):mCyberObstacles(channel_name.c_str()){}
CW_Obstacles::~CW_Obstacles(){}

void CW_Obstacles::set_null_obstacles()
{
    static uint64_t  pre_frame_counter = 0;
    static uint8_t  cnt_lost = 10;

    if (pre_frame_counter == frame_counter)
    {
        if (0 != cnt_lost)
        {
            cnt_lost--;
        }
        else
        {
            mpCyberObstacles->mObstacleSize = 0;
            mpCyberObstacles->publish(frame++);
        }
    }
    else
    {
        pre_frame_counter = frame_counter;
        cnt_lost = 10;
    }
}

bool CW_Obstacles::set_obstacles(int& obj_size, CyberWriterObstacles::Obstacle* obj_list)
{
    static int lastFrame = 0;
    std::unique_ptr<SimOne_Data_Obstacle> pGroundTruth = std::make_unique<SimOne_Data_Obstacle>();

    if (!SimOneAPI::GetGroundTruth("0", pGroundTruth.get()))
    {
        std::cout << "[TaskCyberObstacles] GetGroundTruth Failed!" << std::endl;
        return false;
    }
    if (pGroundTruth->frame == lastFrame)
    {
        return false;
    }
    lastFrame = pGroundTruth->frame;

    obj_size = pGroundTruth->obstacleSize;
    for(int i=0; i< obj_size; i++)
    {
        SimOne_Data_Obstacle_Entry& obj = pGroundTruth->obstacle[i];
        obj_list[i].id = obj.viewId;

        // apollo/modules/perception/proto/perception_obstacle.proto
        switch(obj.type)
        {
            case ESimOne_Obstacle_Type_Car:
            case ESimOne_Obstacle_Type_Truck:
            case ESimOne_Obstacle_Type_Bus:
            case ESimOne_Obstacle_Type_SpecialVehicle:
                obj_list[i].type = 5;
                break;
            case ESimOne_Obstacle_Type_Pedestrian:
                obj_list[i].type = 3;
                break;
            case ESimOne_Obstacle_Type_Bicycle:
            case ESimOne_Obstacle_Type_Rider:
            case ESimOne_Obstacle_Type_Motorcycle:
                obj_list[i].type = 4;
                break;
            default:
                obj_list[i].type = 0;
        }

        obj_list[i].theta = obj.oriZ;
        obj_list[i].posX = obj.posX;
        obj_list[i].posY = obj.posY;
        obj_list[i].posZ = obj.posZ;
        obj_list[i].velX = obj.velX;
        obj_list[i].velY = obj.velY;
        obj_list[i].velZ = obj.velZ;
        obj_list[i].length = obj.length;
        obj_list[i].width = obj.width;
        obj_list[i].height = obj.height;
        obj_list[i].polygonPointSize = 0;
        obj_list[i].confidence = 1;
        obj_list[i].confidenceType = 1;
    }

    return true;
}

// void CW_Obstacles::task_obstacles()
// {
//     static uint64_t  frame = 0;
//     mCyberObstacles.mObstacleSize = 0;
//     memset(mCyberObstacles.mObstacleList, 0, sizeof(CyberWriterObstacles::Obstacle) * OBSTACLE_MAX_SIZE);
// 
//     if (set_obstacles(mCyberObstacles.mObstacleSize, mCyberObstacles.mObstacleList))
//     {
//         mCyberObstacles.publish(frame++);
//     }
// }

void CW_Obstacles::set_obstacles_cb(const char* mainVehicleId, SimOne_Data_Obstacle *pObstacle)
{
    mpCyberObstacles->mObstacleSize = 0;
    memset(mpCyberObstacles->mObstacleList, 0, sizeof(CyberWriterObstacles::Obstacle) * OBSTACLE_MAX_SIZE);

    mpCyberObstacles->mObstacleSize = pObstacle->obstacleSize;
    for (int i = 0; i < pObstacle->obstacleSize; i++)
    {
        SimOne_Data_Obstacle_Entry &obj = pObstacle->obstacle[i];
        mpCyberObstacles->mObstacleList[i].id = obj.viewId;

        // apollo/modules/perception/proto/perception_obstacle.proto
        switch (obj.type)
        {
        case ESimOne_Obstacle_Type_Car:
        case ESimOne_Obstacle_Type_Truck:
        case ESimOne_Obstacle_Type_Bus:
        case ESimOne_Obstacle_Type_SpecialVehicle:
            mpCyberObstacles->mObstacleList[i].type = 5;
            break;
        case ESimOne_Obstacle_Type_Pedestrian:
            mpCyberObstacles->mObstacleList[i].type = 3;
            break;
        case ESimOne_Obstacle_Type_Bicycle:
        case ESimOne_Obstacle_Type_Rider:
        case ESimOne_Obstacle_Type_Motorcycle:
            mpCyberObstacles->mObstacleList[i].type = 4;
            break;
        default:
            mpCyberObstacles->mObstacleList[i].type = 0;
        }

        mpCyberObstacles->mObstacleList[i].theta = obj.oriZ;
        mpCyberObstacles->mObstacleList[i].posX = obj.posX;
        mpCyberObstacles->mObstacleList[i].posY = obj.posY;
        mpCyberObstacles->mObstacleList[i].posZ = obj.posZ;
        mpCyberObstacles->mObstacleList[i].velX = obj.velX;
        mpCyberObstacles->mObstacleList[i].velY = obj.velY;
        mpCyberObstacles->mObstacleList[i].velZ = obj.velZ;
        mpCyberObstacles->mObstacleList[i].length = obj.length;
        mpCyberObstacles->mObstacleList[i].width = obj.width;
        mpCyberObstacles->mObstacleList[i].height = obj.height;
        mpCyberObstacles->mObstacleList[i].polygonPointSize = 0;
        mpCyberObstacles->mObstacleList[i].confidence = 1;
        mpCyberObstacles->mObstacleList[i].confidenceType = 1;
    }
    frame_counter = frame;
    mpCyberObstacles->publish(frame++);
}

void CW_Obstacles::task_obstacles()
{
    mpCyberObstacles = &mCyberObstacles;
    if (!SimOneAPI::SetGroundTruthUpdateCB(set_obstacles_cb))
    {
        printf("\033[1m\033[31m[task_obstacles]: SetSimOneGroundTruthCB Failed!\033[0m\n");
    }
}
