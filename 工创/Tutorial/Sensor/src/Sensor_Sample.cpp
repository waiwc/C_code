#include "Sensor_Sample.h"

dumper sensorapi_sample::dbg_data;

sensorapi_sample::sensorapi_sample(const char* mv_id):mainVehicleId(mv_id){}
sensorapi_sample::~sensorapi_sample(){}

void sensorapi_sample::InitSimOneAPI(bool isJoinTimeLoop, const char *serverIP)
{
	// const char *mainVehicleId = "0", bool isFrameSync = false, const char *serverIP = "127.0.0.1", int port = 23789, void (*startCase)() = 0, void (*endCase)() = 0, int registerNodeId = 0;
	SimOneAPI::InitSimOneAPI(mainVehicleId.c_str(), isJoinTimeLoop, serverIP);
}

int64_t sensorapi_sample::getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000; // milliseconds
    // return tv.tv_sec * 1e6 + tv.tv_usec; // microseconds
}

void sensorapi_sample::Test_GetEnvironment()
{
	std::unique_ptr<SimOne_Data_Environment> pEnvironment = std::make_unique<SimOne_Data_Environment>();
	bool flag = SimOneAPI::GetEnvironment(pEnvironment.get());
	if (flag)
	{
		dbg_data.dump_environment(pEnvironment.get());
	}
	else
	{
		std::cout << "GetEnvironment Failed!" << std::endl;
		return;
	}
}

void sensorapi_sample::Test_GPS(bool IsCallBackMode)
{
	if (IsCallBackMode) {
		auto function = []( const char* mainVehicleId, SimOne_Data_Gps *pGps){
				dbg_data.dump_gps(mainVehicleId, pGps);
		};
		 SimOneAPI::SetGpsUpdateCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
		int lastFrame = 0;
		while(1)
		{
			bool flag = SimOneAPI::GetGps(mainVehicleId.c_str(), pGps.get());
			if (flag && pGps->frame != lastFrame)
			{
				lastFrame  = pGps->frame;
				dbg_data.dump_gps(mainVehicleId.c_str(), pGps.get());
			}
			if (!flag)
			{
				std::cout<<"Get GPS Fail"<< std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

void sensorapi_sample::Test_GetGroundTruth(bool IsCallBackMode)
{
	if (IsCallBackMode) {
		auto function = [](const char* mainVehicleId, SimOne_Data_Obstacle *pObstacle) {
			dbg_data.dump_ground_truth(mainVehicleId, pObstacle);
		};
		SimOneAPI::SetGroundTruthUpdateCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_Obstacle> pDetections = std::make_unique<SimOne_Data_Obstacle>();
		int lastFrame = 0;
		while (true) {
			bool flag = SimOneAPI::GetGroundTruth(mainVehicleId.c_str(),  pDetections.get());
			if (flag && pDetections->frame != lastFrame)
			{
				lastFrame  = pDetections->frame;
				dbg_data.dump_ground_truth(mainVehicleId.c_str(), pDetections.get());
			}
			if (!flag)
			{
				std::cout << "GetGroundTruth Failed!" << std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}
}

void sensorapi_sample::Test_RadarDetection(bool IsCallBackMode)
{
	if (IsCallBackMode) {
		auto function = [](const char* mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection *pDetections)
		{
			dbg_data.dump_radar_detection(mainVehicleId, sensorId, pDetections);
		};
		SimOneAPI::SetRadarDetectionsUpdateCB(function);
	}
	else
	{
		std::unique_ptr<SimOne_Data_RadarDetection> pDetections = std::make_unique<SimOne_Data_RadarDetection>();
		int lastFrame = 0;
		while (true)
		{
			bool flag = SimOneAPI::GetRadarDetections(mainVehicleId.c_str(), "objectBasedRadar1", pDetections.get());
			if (flag && pDetections->frame != lastFrame)
			{
				lastFrame  = pDetections->frame;
				dbg_data.dump_radar_detection(mainVehicleId.c_str(), "objectBasedRadar1", pDetections.get());
			}
			if (!flag)
			{
				std::cout << "GetRadarDetections Failed!" << std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

void sensorapi_sample::Test_UltrasonicRadars(bool IsCallBackMode)
{
	if (IsCallBackMode) {
		auto function = [](const char* mainVehicleId, SimOne_Data_UltrasonicRadars* pUltrasonics)
		{
			dbg_data.dump_ultrasonic_radars(mainVehicleId, pUltrasonics);
		};
		SimOneAPI::SetUltrasonicRadarsCB(function);
	}
	else
	{
		std::unique_ptr<SimOne_Data_UltrasonicRadars> pDetections = std::make_unique<SimOne_Data_UltrasonicRadars>();
		int lastFrame = 0;
		while (true)
		{
			bool flag = SimOneAPI::GetUltrasonicRadars(mainVehicleId.c_str(), pDetections.get());
			if (flag && pDetections->frame != lastFrame)
			{
				lastFrame  = pDetections->frame;
				dbg_data.dump_ultrasonic_radars(mainVehicleId.c_str(), pDetections.get());
			}
			if (!flag)
			{
				std::cout << "GetUltrasonicRadars Failed!" << std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

void sensorapi_sample::Test_GetSensorDetections(bool IsCallBackMode)
{
	if (IsCallBackMode)
	{
		auto function = [](const char* MainVehicleID, const char* sensorId, SimOne_Data_SensorDetections* pGroundtruth)
		{
			dbg_data.dump_sensor_detections(MainVehicleID, sensorId, pGroundtruth);
		};
		SimOneAPI::SetSensorDetectionsUpdateCB(function);
	}
	else
	{
		std::unique_ptr<SimOne_Data_SensorDetections> pGroundtruth = std::make_unique<SimOne_Data_SensorDetections>();
		int lastFrame = 0;
		while (true)
		{
			// "sensorFusion1" "objectBasedCamera1" "objectBasedLidar1" "perfectPerception1"
			bool flag = SimOneAPI::GetSensorDetections(mainVehicleId.c_str(), "perfectPerception1", pGroundtruth.get());
			if (flag && pGroundtruth->frame != lastFrame)
			{
				lastFrame = pGroundtruth->frame;
				dbg_data.dump_sensor_detections(mainVehicleId.c_str(), "perfectPerception1", pGroundtruth.get());
			}
			if (!flag)
			{
				std::cout << "GetSensorDetections Failed!" << std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

void sensorapi_sample::Test_GetSensorConfigurations()
{
	std::unique_ptr<SimOne_Data_SensorConfigurations> pConfigs = std::make_unique<SimOne_Data_SensorConfigurations>();
	while (true)
	{
		bool flag = SimOneAPI::GetSensorConfigurations(mainVehicleId.c_str(), pConfigs.get());
		if (flag)
		{
			dbg_data.dump_sensor_configurations(mainVehicleId.c_str(), pConfigs.get());
		}
		else
		{
			std::cout << "there is no sensor config in current vehicle!!!" << std::endl;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(30));
	}
}

void sensorapi_sample::Test_SetEnvironment()
{
	SimOne_Data_Environment Environment; // std::unique_ptr<SimOne_Data_Environment> pEnvironment = std::make_unique<SimOne_Data_Environment>();
	Environment.timeOfDay = 1000;
	Environment.heightAngle = 90;
	Environment.directionalLight = 0.5f;
	Environment.ambientLight = 0.5f;
	Environment.artificialLight = 0.5f;
	Environment.cloudDensity = 0.5f;
	Environment.fogDensity = 0.5f;
	Environment.rainDensity = 0.5f;
	Environment.snowDensity = 0.5f;
	Environment.groundHumidityLevel = 0.5f;
	Environment.groundDirtyLevel = 0.5f;
       if(!SimOneAPI::SetEnvironment(&Environment))
	{
        	std::cout << "SetEnvironment Failed!" << std::endl; 
	}
	else
	{
		dbg_data.dump_environment(&Environment);
	}
}

void sensorapi_sample::Test_SensorLaneInfo(bool IsCallBackMode)
{
	if (IsCallBackMode) {
		auto function = [](const char* mainVehicleId, const char* sensorId, SimOne_Data_LaneInfo *pDetections) {
			dbg_data.dump_sensor_laneInfo(mainVehicleId, sensorId, pDetections);
		};
		SimOneAPI::SetSensorLaneInfoCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_LaneInfo> pDetections = std::make_unique<SimOne_Data_LaneInfo>();
		int lastFrame = 0;
		while (true) {
			bool flag = SimOneAPI::GetSensorLaneInfo(mainVehicleId.c_str(), "sensorFusion1", pDetections.get());
			if (flag && pDetections->frame != lastFrame)
			{
				lastFrame = pDetections->frame;
				dbg_data.dump_sensor_laneInfo(mainVehicleId.c_str(), "sensorFusion1", pDetections.get());
			}
			if (!flag)
			{
				std::cout << "GetSensorLaneInfo Failed!" << std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

int main(int argc, char* argv[])
{
	const char *mv_id = "0";
	sensorapi_sample st(mv_id);

	bool isJoinTimeLoop = false;
	const char *serverIP = "127.0.0.1";
	st.InitSimOneAPI(isJoinTimeLoop, serverIP);
	st.Test_GetSensorDetections(true);
	while (true)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	return 0;
}
