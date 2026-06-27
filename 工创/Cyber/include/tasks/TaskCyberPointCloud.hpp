#ifndef TASKCYBERPOINTCLOUD_HPP_
#define TASKCYBERPOINTCLOUD_HPP_

#include <string.h>
#include <unistd.h>
#include <vector>
#include <mutex>

#include "SimOneStreamingAPI.h"
#include "CyberWriterPointCloud.hpp"

typedef struct
{
    std::string ip = "127.0.0.1";
    unsigned short infoPort = 7788;
    unsigned short port = 6699;
    std::string frameId = "velodyne128";
} PCD_Param_T;

class CW_PointCloud
{
    public:
        CW_PointCloud(std::string channel_name);
        ~CW_PointCloud();
        void set_params(PCD_Param_T& param);
        static void set_point_cloud(SimOne_Streaming_Point_Cloud *pPointCloud);
        void task_point_cloud();
    private:
        CyberWriterPointCloud mCyberPointCloud;
        static CyberWriterPointCloud* pCyberPointCloud;
        static PCD_Param_T mParam;
        static pcl::PointCloud<pcl::PointXYZI>::Ptr pData; 
};

#endif
