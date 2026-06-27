#ifndef DUMPER_H
#define DUMPER_H

#include "Service/SimOneIOStruct.h"
#include <iostream>
using namespace std;

class dumper
{
public:
	dumper();
	~dumper();

	void dump_gps(const char* mainVehicleId, SimOne_Data_Gps* pData);
	void dump_ground_truth(const char* mainVehicleId, SimOne_Data_Obstacle* pData);
	void dump_radar_detection(const char* mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection * pData);
	void dump_ultrasonic_radar(const char* mainVehicleId, const char* sensorId, SimOne_Data_UltrasonicRadar* pData);
	void dump_ultrasonic_radars(const char* mainVehicleId, SimOne_Data_UltrasonicRadars* pData);
	void dump_sensor_detections(const char* mainVehicleId, const char* sensorId, SimOne_Data_SensorDetections* pData);
	void dump_sensor_configurations(const char* mainVehicleId, SimOne_Data_SensorConfigurations* pData);
	void dump_environment(SimOne_Data_Environment* pData);
	void dump_traffic_light(const char* mainVehicleId, int opendriveLightId, SimOne_Data_TrafficLight* pData);
	void dump_sensor_laneInfo(const char* mainVehicleId, const char* sensorId, SimOne_Data_LaneInfo* pData);
};

#endif