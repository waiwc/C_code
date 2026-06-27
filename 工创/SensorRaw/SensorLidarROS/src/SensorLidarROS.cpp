#include <SimOneStreamingAPI.h>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>

std::string gIP = "127.0.0.1";
unsigned short gInfoPort = 7788;
unsigned short gPort = 6699;
std::string gTopic = "/sensing/lidar/pointcloud";
std::string gFrameId = "lidar";
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
	if (argc >= 5)
	{
		gTopic = argv[4];
		gFrameId = argv[5];
	}
	printf("IP: %s;  Port: %d; Info Port: %d\n", gIP.c_str(), gPort, gInfoPort);
	printf("Topic: %s;  FrameId: %s\n", gTopic.c_str(), gFrameId.c_str());

	SimOneAPI::SetStreamingPointCloudUpdateCB(gIP.c_str(), gPort, gInfoPort, dataPointCloudCallback);

	int ros_argc = 0;
	ros::init(ros_argc,nullptr,"SimOne");
    	ros::NodeHandle nh;
	ros::Publisher point_pub = nh.advertise<sensor_msgs::PointCloud2> (gTopic, 10);
	ros::Rate loop_rate(20);

  static std::string sNames[] = {"x", "y", "z", "intensity"};
  static int sOffsets[] = {0, 4, 8, 12};
  static int sDatatypes[] = {7, 7, 7, 7};


	int lastFrame = 0;
	while (ros::ok())
	{
		{
			std::lock_guard<std::mutex> lock(gDataPointCloudMutex);
			if (gDataPointCloud.frame != lastFrame && gDataPointCloud.pointStep == sizeof(SimOne_Streaming_Point_XYZI)) {
				int pointCount = gDataPointCloud.pointCloudDataSize / gDataPointCloud.pointStep;
				pcl::PCLPointCloud2 pointCloud_msg;	
				
				pointCloud_msg.header.seq = gDataPointCloud.frame;		// sequence number long long
				pointCloud_msg.header.stamp = gDataPointCloud.timestamp * 1e3; 
				pointCloud_msg.header.frame_id = gFrameId;
				pointCloud_msg.point_step = gDataPointCloud.pointStep;
				pointCloud_msg.row_step = gDataPointCloud.pointStep * gDataPointCloud.width;
				pointCloud_msg.height = gDataPointCloud.height;
				pointCloud_msg.width = gDataPointCloud.width;

				for (int i = 0; i < 4; i++)
				{
					pcl::PCLPointField field;
					field.name = sNames[i].c_str();
					field.offset = sOffsets[i];
					field.datatype = sDatatypes[i];
					field.count = 1;
					pointCloud_msg.fields.push_back(field);
				}

				pointCloud_msg.is_bigendian = false;

				pointCloud_msg.data.resize(gDataPointCloud.pointCloudDataSize);
				memcpy(&pointCloud_msg.data[0], gDataPointCloud.pointCloudData, gDataPointCloud.pointCloudDataSize);
				pointCloud_msg.is_dense = true;

				printf("[%d]PointCloud got with data size %d, TimeStamp %lld\n", gDataPointCloud.frame, gDataPointCloud.pointCloudDataSize, gDataPointCloud.timestamp );
				point_pub.publish(pointCloud_msg);

				gDataPointCloud.frame = lastFrame;
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
    }
    return 0;
}

