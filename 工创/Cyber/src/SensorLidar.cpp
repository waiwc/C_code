// #include <SimOneStreamingAPI.h>
// #include "CyberWriterPointCloud.hpp"
// #include <string.h>
// #include <unistd.h>
// #include <vector>
// #include <mutex>

#include "TaskCyberPointCloud.hpp"

int main(int argc, char* argv[])
{

  std::string mPCDChannel = "/apollo/sensor/lidar128/compensator/PointCloud2";

  CW_PointCloud mPointCloud(mPCDChannel);
PCD_Param_T param;
param.ip = "127.0.0.1";
param.infoPort = 7788;
param.port = 6677;
param.frameId = "veloney128";
mPointCloud.set_params(param);


  mPointCloud.task_point_cloud();

  while (1)
  {
	  sleep(1);
  }

  return 0;
}











/*

std::string gIP = "127.0.0.1";
unsigned short gInfoPort = 7788;
unsigned short gPort = 6699;
std::string gChannel = "/apollo/sensor/lidar128/compensator/PointCloud2";
std::string gFrameId = "velodyne128";
SimOne_Streaming_Point_Cloud gDataPointCloud;
std::mutex	gDataPointCloudMutex;

void dataPointCloudCallback(SimOne_Streaming_Point_Cloud *pPointCloud)
{
	std::lock_guard<std::mutex> lock(gDataPointCloudMutex);
	gDataPointCloud.frame = pPointCloud->frame;
	gDataPointCloud.timestamp = pPointCloud->timestamp;
	gDataPointCloud.height = pPointCloud->height;
	gDataPointCloud.width = pPointCloud->width;
	gDataPointCloud.pointStep = pPointCloud->pointStep;
	gDataPointCloud.pointCloudDataSize = pPointCloud->pointCloudDataSize;
	memcpy(&gDataPointCloud.pointCloudData, &pPointCloud->pointCloudData, pPointCloud->pointCloudDataSize);
}

int main(int argc, char* argv[])
{
	if (argc > 3)
	{
		gIP = argv[1];
		gPort = atoi(&*argv[2]);
		gInfoPort = atoi(&*argv[3]);
	}
	if(argc > 5){
		gChannel = argv[4];
		gFrameId = argv[5];
	}
	printf("IP: %s;  Port: %d; Info Port: %d\n", gIP.c_str(), gPort, gInfoPort);
	printf("FrameId: %s; Channel: %s\n", gFrameId.c_str(), gChannel.c_str());
	CyberWriterPointCloud cyberPointCloud(gChannel);
	pcl::PointCloud<pcl::PointXYZI>::Ptr pclCloud (new pcl::PointCloud<pcl::PointXYZI>);

	SimOneAPI::SetStreamingPointCloudUpdateCB(gIP.c_str(), gPort, gInfoPort, dataPointCloudCallback);

	int lastFrame = 0;
	while (true)
	{
		{
			std::lock_guard<std::mutex> lock(gDataPointCloudMutex);
			if (gDataPointCloud.frame != lastFrame && gDataPointCloud.pointStep == sizeof(SimOne_Streaming_Point_XYZI)) {
				
				int pointCount = gDataPointCloud.pointCloudDataSize / gDataPointCloud.pointStep;
				pclCloud->width = pointCount;
				pclCloud->height = 1;
				pclCloud->reserve(pointCount);
				memcpy(&pclCloud->points[0], gDataPointCloud.pointCloudData, gDataPointCloud.pointCloudDataSize);
				cyberPointCloud.publish(gFrameId, pclCloud);
				gDataPointCloud.frame = lastFrame;
			}
		}
		usleep(1000);
    }

    return 0;
}
	*/
