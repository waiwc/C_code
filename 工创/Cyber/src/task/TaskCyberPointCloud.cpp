#include "TaskCyberPointCloud.hpp"

pcl::PointCloud<pcl::PointXYZI>::Ptr CW_PointCloud::pData(new pcl::PointCloud<pcl::PointXYZI>);
CyberWriterPointCloud* CW_PointCloud::pCyberPointCloud = nullptr;
PCD_Param_T CW_PointCloud::mParam;

CW_PointCloud::CW_PointCloud(std::string channel_name):mCyberPointCloud(channel_name.c_str()){}

CW_PointCloud::~CW_PointCloud(){}

void CW_PointCloud::set_params(PCD_Param_T& param)
{
    mParam.ip = param.ip;
    mParam.infoPort = param.infoPort;
    mParam.port = param.port;
    mParam.frameId = param.frameId;
}

void CW_PointCloud::set_point_cloud(SimOne_Streaming_Point_Cloud *pPointCloud)
{
    if (pPointCloud->pointStep == sizeof(SimOne_Streaming_Point_XYZI))
    {
        // pPointCloud->frame;
        // pPointCloud->timestamp;
        // pPointCloud->height;
        // pPointCloud->width;
        int pointCount = pPointCloud->pointCloudDataSize / pPointCloud->pointStep;
        pData->width = pointCount;
        pData->height = 1;
        pData->reserve(pointCount);
        memcpy(&pData->points[0], &pPointCloud->pointCloudData, pPointCloud->pointCloudDataSize);

        pCyberPointCloud->publish(mParam.frameId, pData);
    }
    else
    {
        std::cout << "[CW_PointCloud] pointStep =" << pPointCloud->pointStep << " not equal to SimOne_Streaming_Point_XYZI size (" << sizeof(SimOne_Streaming_Point_XYZI) << ")" << std::endl;
    }
}

void CW_PointCloud::task_point_cloud()
{
    pCyberPointCloud = &mCyberPointCloud;
	SimOneAPI::SetStreamingPointCloudUpdateCB(mParam.ip.c_str(), mParam.port, mParam.infoPort, set_point_cloud);
}
