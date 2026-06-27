#include <SimOneStreamingAPI.h>
#include "visualization/cloud_viewer.h"
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>

std::string gIP = "127.0.0.1";
unsigned short gInfoPort = 7788;
unsigned short gPort = 6699;
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
	if (argc >= 3)
	{
		gIP = argv[1];
		gPort = atoi(&*argv[2]);
		gInfoPort = atoi(&*argv[3]);
	}
	printf("IP: %s;  Port: %d; Info Port: %d\n", gIP.c_str(), gPort, gInfoPort);

	SimOneAPI::SetStreamingPointCloudUpdateCB(gIP.c_str(), gPort, gInfoPort, dataPointCloudCallback);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("51WORLD LiDAR Viewer"));
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr viewCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGBA>(viewCloud, "cloud");
	viewer->addCoordinateSystem(0.1);
	viewer->initCameraParameters();
	viewer->setCameraPosition(-8, 0, 4.5, 1, 0, 0);

	int lastFrame = 0;
	while (1)
	{
		{
			std::lock_guard<std::mutex> lock(gDataPointCloudMutex);
			if (gDataPointCloud.frame != lastFrame && gDataPointCloud.pointStep == sizeof(SimOne_Streaming_Point_XYZI)) {
				viewCloud->points.clear();
				int pointCount = gDataPointCloud.pointCloudDataSize / gDataPointCloud.pointStep;
				viewCloud->points.resize(pointCount);
				SimOne_Streaming_Point_XYZI* pPoint = (SimOne_Streaming_Point_XYZI*)gDataPointCloud.pointCloudData;
				for (int i = 0; i < pointCount; i++, pPoint++) {
					auto& viewPoint = viewCloud->points[i];
					viewPoint.x = pPoint->x;
					viewPoint.y = pPoint->y;
					viewPoint.z = pPoint->z;
					int intensity = (int)(pPoint->intensity * 255);
					if (intensity <= 33) {
						viewPoint.r = 0;
						viewPoint.g = (int)(7.727 * intensity);
						viewPoint.b = 255;
					}
					else if (intensity > 33 && intensity <= 66) {
						viewPoint.r = 0;
						viewPoint.g = 255;
						viewPoint.b = (int)(255 - 7.727 * (intensity - 34));
					}
					else if (intensity > 66 && intensity <= 100) {
						viewPoint.r = (int)(7.727 * (intensity - 67));
						viewPoint.g = 255;
						viewPoint.b = 0;
					}
					else if (intensity > 100 && intensity <= 255) {
						viewPoint.r = 255;
						// (100, 255] => (0, 33]
						viewPoint.g = (int)(255 - 7.727 * (intensity - 100) / 4.697);
						viewPoint.b = 0;
					}

					viewPoint.a = 255;
				}
				viewer->removeAllPointClouds();
#if defined __WIN32 || defined WIN32
				viewer->addPointCloud(viewCloud);
#else
				viewer->updatePointCloud(viewCloud);
#endif
				gDataPointCloud.frame = lastFrame;
			}
		}
		viewer->spinOnce(1);
    }
    return 0;
}

