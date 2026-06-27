
#include "SimOneServiceAPI.h"
#include "SimOneSensorAPI.h"
#include "SimOneHDMapAPI.h"
#include "SimOneEvaluationAPI.h"
#include "SSD/SimPoint3D.h"
#include "UtilDriver.h"
#include "UtilMath.h"
#include "C:\sim-one\SimOneAPI\Tutorial\HDMap\include\SampleGetNearMostLane.h"
#include "C:\sim-one\SimOneAPI\Tutorial\HDMap\include\SampleGetLaneST.h"
#include <iomanip> 
#include <iostream>
#include <ctime>
#include <memory>
#include <C:\Sim-One\SimOneAPI\cpp-httplib\httplib.h>/////////////
#include <chrono>////////////
#include "C:\Sim-One\SimOneAPI\cpp-httplib\TestController.h"///////////////
#include "C:\Sim-One\SimOneAPI\cpp-httplib\HttpServer.h" //////////
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <fstream> 
#include <mutex>

double actualSystemTimeYY = 0;

float totalDistance = 0.0f;       // 累计总路程
double totalSpeed = 0.0;          // 累计速度总和
double frameAccumulatedTime = 0.0; // 各帧累加估算时间
int frameCount = 0;               // 帧计数



std::unordered_set<std::string> received_data; // 存储接收到的数据
std::mutex data_mutex; // 互斥锁，确保数据线程安全
//////////////
namespace httplib {

	// 定义 HttpServer 的静态成员变量
	httplib::MyHttpServerThreadPool* HttpServer::serverPool = nullptr;
	std::atomic<int> HttpServer::poolUseNum(0);

#if HTTPSERVER_AUTO_CONTROLLER
	// 定义 AutoBashController 的静态成员变量
	std::unordered_map<std::string, std::pair<HttpServerRequestType, httplib::Server::Handler>> AutoBashController::__autoBindFuncMap;
	std::mutex AutoBashController::__autoMapResetMtx;
#endif

} // namespace httplib

using namespace httplib;

///////////////////////////
float targetSteering;     // 目标方向值
float currentSteering;    // 当前方向值
const float steeringSpeed = 0.05f;  // 控制回正速度的系数
int main()
{

	////////////////////////////////////////////////////////////////
	httplib::HttpServer* server = new httplib::HttpServer("0.0.0.0", 9899);

	// 创建 TestController 实例
	TestController* controller = new TestController();
	server->addController(controller);

	// 启动服务器监听线程
	std::cout << "HTTP Server is starting on port 9899..." << std::endl;
	server->listenInThread();

	/////////////////////////////////////////////////////////////////////////////
	std::ofstream outFile("output.csv");
	if (!outFile.is_open()) {
		std::cerr << "Failed to open output file." << std::endl;
		return -1;
	}//写入路径
	bool inAEBState = false;
	int timeout = 20;
	bool isSimOneInitialized = false;
	const char* MainVehicleId = "0";
	bool isJoinTimeLoop = true;
	SimOneAPI::InitSimOneAPI(MainVehicleId, isJoinTimeLoop);
	SimOneAPI::SetDriverName(MainVehicleId, "Manual");
	SimOneAPI::InitEvaluationServiceWithLocalData(MainVehicleId);
	while (true) {
		if (SimOneAPI::LoadHDMap(timeout)) {

			break;
		}

	}

	//输入点的容器
	SSD::SimPoint3DVector inputPoints;
	//存储车道信息的指针
	std::unique_ptr<SimOne_Data_WayPoints> pWayPoints = std::make_unique<SimOne_Data_WayPoints>();

	//主车路点数据
	if (SimOneAPI::GetWayPoints(MainVehicleId, pWayPoints.get()))
	{
		// 提取主车道点的坐标并添加到输入点容器中
		for (size_t i = 0; i < pWayPoints->wayPointsSize; ++i) {
			SSD::SimPoint3D inputWayPoints(pWayPoints->wayPoints[i].posX, pWayPoints->wayPoints[i].posY, 0);
			inputPoints.push_back(inputWayPoints);
		}
	}
	else {

		return -1;
	}



	// 定义用于保存初始点的坐标变量
	float initialPosX = 0.0f;
	float initialPosY = 0.0f;
	float initialoriZ = 0.0f;
	float relativeDistance;


	std::unique_ptr<SimOne_Data_Gps> pGps01 = std::make_unique<SimOne_Data_Gps>();
	if (!SimOneAPI::GetGps(MainVehicleId, pGps01.get())) {

	}

	initialPosX = pGps01->posX;
	initialPosY = pGps01->posY;
	initialoriZ = pGps01->oriZ;
	float lastPrintPosX = initialPosX;
	float lastPrintPosY = initialPosY;
	float lastPrintPosX2 = initialPosX;
	float lastPrintPosY2 = initialPosY;

	//目标路径的容器
	SSD::SimPoint3DVector targetPath;
	if (pWayPoints->wayPointsSize >= 2) {
		SSD::SimVector<int> indexOfValidPoints;

		//生成主车路线
		if (!SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath)) {

			return -1;
		}
	}
	else if (pWayPoints->wayPointsSize == 1) {
		//只有一个路点，获取最近的车道并初始化路线
		SSD::SimString laneIdInit = SampleGetNearMostLane(inputPoints[0]);
		HDMapStandalone::MLaneInfo laneInfoInit;
		if (!SimOneAPI::GetLaneSample(laneIdInit, laneInfoInit)) {

			return -1;
		}
		else {
			targetPath = laneInfoInit.centerLine;
		}
	}
	else {

		return -1;
	}


	std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();  // 记录开始时间
	std::chrono::time_point<std::chrono::steady_clock> last_report;
	last_report = start;
	std::chrono::time_point<std::chrono::steady_clock> nowtt;
	std::chrono::duration<double> elapsed;
	std::chrono::duration<double> since_last_report;
	outFile << std::fixed << std::setprecision(2) << 0.0 << ", "
		<< std::setprecision(2) << initialPosX << ", " << initialPosY << ", "
		<< std::setprecision(2) << 0.0 << ", " // 初始加速度为0
		<< std::setprecision(5) << initialoriZ << ", " // 初始方向为0
		<< std::setprecision(2) << 0.0 << ", " // 相对距离初始为0
		<< std::setprecision(2) << 0.0 << ", "     // 初始速度为0
		<< std::endl;



	// 使用 chrono 记录程序的实际运行时间
	auto startSystemTime = std::chrono::steady_clock::now();
	while (true) {
		int frame = SimOneAPI::Wait();


		//获取GPS信息
		std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
		if (SimOneAPI::GetGps(MainVehicleId, pGps.get())) {
			// 计算当前点到初始点的距离
			relativeDistance = std::sqrt(std::pow(pGps->posX - initialPosX, 2) + std::pow(pGps->posY - initialPosY, 2));

		}
		float currentPosX = pGps->posX;
		float currentPosY = pGps->posY;
		float speed = std::sqrt(pGps->velX * pGps->velX + pGps->velY * pGps->velY + pGps->velZ * pGps->velZ);

		// 计算当前位置和上一个位置之间的距离
		float distance = std::sqrt(std::pow(currentPosX - lastPrintPosX2, 2) + std::pow(currentPosY - lastPrintPosY2, 2));
		totalDistance += distance;     // 累加到总路程
		totalSpeed += speed;           // 累加速度总和
		frameCount++;                  // 帧计数

		// 根据速度估算当前帧的行驶时间并累加,用这个可以得到最准确的帧时间，但是停下来后就无法累加了
		//if (speed > 0) {
		//	frameAccumulatedTime += (distance / speed);
		//}
		// 计算总路程/平均速度的估算时间
		double averageSpeed = frameCount > 0 ? totalSpeed / frameCount : 0.0;  // 平均速度
		double distanceSpeedEstimatedTime = averageSpeed > 0 ? totalDistance / averageSpeed : 0.0; // 估算的总时间，通过总路程和平均速度得到，不会在速度为0时停止
				// 更新上一个位置为当前点
		lastPrintPosX2 = currentPosX;
		lastPrintPosY2 = currentPosY;
		//// 系统实际运行时间
		//auto endSystemTime = std::chrono::steady_clock::now();
		//std::chrono::duration<double> systemElapsedTime = endSystemTime - startSystemTime;
		//double actualSystemTime = systemElapsedTime.count(); // 系统实际运行时间

		//if (actualSystemTime- actualSystemTimeYY>=1) {
		//	// 输出三种时间估算结果
		//	std::cout << "========= Time Estimation Results =========" << std::endl;
		//	std::cout << "Total Distance Traveled: " << totalDistance << " meters" << std::endl;
		//	std::cout << "Average Speed: " << averageSpeed << " m/s" << std::endl;
		//	std::cout << "1. Frame-Accumulated Estimated Time: " << frameAccumulatedTime << " seconds" << std::endl;
		//	std::cout << "2. Distance/Average Speed Estimated Time: " << distanceSpeedEstimatedTime << " seconds" << std::endl;
		//	std::cout << "3. Actual System Time: " << actualSystemTime << " seconds" << std::endl;
		//	actualSystemTimeYY = actualSystemTime;
		//}


		float printDistanceThreshold = 0.2f; // 几米打印一次
		nowtt = std::chrono::steady_clock::now();
		//elapsed = nowtt - start;  // 计算从开始的总运行时间
		float distanceSinceLastPrint = std::sqrt(std::pow(pGps->posX - lastPrintPosX, 2) + std::pow(pGps->posY - lastPrintPosY, 2));

		// 每当距离阈值达到3米，打印数据并更新上次打印位置
		if (distanceSinceLastPrint >= printDistanceThreshold) {
			outFile << std::fixed << std::setprecision(2) << distanceSpeedEstimatedTime /*elapsed.count()*/ << ", "
				<< std::setprecision(2) << pGps->posX << ", " << pGps->posY << ", "
				<< std::setprecision(2) << "0.00"/*std::sqrt(pGps->accelX * pGps->accelX + pGps->accelY * pGps->accelY + pGps->accelZ * pGps->accelZ)*/ << ", "
				<< std::setprecision(5) << pGps->oriZ << ", "
				<< std::setprecision(2) << relativeDistance << ", "
				<< std::setprecision(2) << std::sqrt(pGps->velX * pGps->velX + pGps->velY * pGps->velY + pGps->velZ * pGps->velZ) << ", "
				<< std::endl;

			// 更新上次打印的位置为当前点
			lastPrintPosX = pGps->posX;
			lastPrintPosY = pGps->posY;
		}
		since_last_report = nowtt - last_report; // 计算从上次报告的时间

	   // if (since_last_report.count() >= 0.2|| elapsed.count()<=0.05) {  // 每隔0.2秒报告一次
	   // 	std::cout << std::fixed << std::setprecision(2) << elapsed.count() << ", "
	   // 		<< std::setprecision(2) << pGps->posX << ", " << pGps->posY << ", "
	   // 		<< std::setprecision(2) << std::sqrt(pGps->accelX * pGps->accelX + pGps->accelY * pGps->accelY + pGps->accelZ * pGps->accelZ) << ", "
	   // 		<< std::setprecision(4) << pGps->oriZ << ", "
	   // 		<< std::setprecision(2) << relativeDistance << ", "
	   // 		<< std::setprecision(2) << std::sqrt(pGps->velX * pGps->velX + pGps->velY * pGps->velY + pGps->velZ * pGps->velZ) << ", "
	   // 		<< std::endl;			
	   // 	last_report = nowtt;  // 更新上次报告时间
	   // }



	   // 根据驻车停止状态进入评价函数
		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop) {
			SimOneAPI::SaveEvaluationRecord();
			outFile.close();
			break;
		}



		//获取障碍物数据
		std::unique_ptr<SimOne_Data_Obstacle> pObstacle = std::make_unique<SimOne_Data_Obstacle>();
		if (!SimOneAPI::GetGroundTruth(MainVehicleId, pObstacle.get())) {

		}

		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Running) {
			if (!isSimOneInitialized) {

				isSimOneInitialized = true;
			}


			//获取主车位置和速度
			SSD::SimPoint3D mainVehiclePos(pGps->posX, pGps->posY, pGps->posZ);
			double mainVehicleSpeed = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ);




			//初始化距离和障碍物索引
			double minDistance = std::numeric_limits<double>::max();
			int potentialObstacleIndex = pObstacle->obstacleSize;
			SSD::SimString mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos);
			SSD::SimString potentialObstacleLaneId = "";
			for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {
				//获取障碍位置和车道
				SSD::SimPoint3D obstaclePos(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, pObstacle->obstacle[i].posZ);
				SSD::SimString obstacleLaneId = SampleGetNearMostLane(obstaclePos);

			}


			auto& potentialObstacle = pObstacle->obstacle[potentialObstacleIndex];
			double obstacleSpeed = UtilMath::calculateSpeed(potentialObstacle.velX, potentialObstacle.velY, potentialObstacle.velZ);



			std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();

			// 使用SimOneDriver控制主车
			SimOneAPI::GetDriverControl(MainVehicleId, pControl.get());
			// 使用互斥锁访问 shared_data
			{
				std::lock_guard<std::mutex> lock(data_mutex);

				float tmp;


				pControl->isManualGear = false;  // 手动挡标志，设置为false表示不使用手动挡
				if (received_data.find("w") != received_data.end()) {
					if (received_data.find("shift") != received_data.end()) {
						tmp = 1.0f;  // 油门踏板位置，设置为0.12表示轻踩油门
					}
					else {
						tmp = 0.3f;  // 油门踏板位置，设置为0.12表示轻踩油门

					}
				}
				else {
					tmp = 0.0f;
				}



				if (received_data.find("space") != received_data.end()) {
					pControl->brake = 1.f;        // 制动踏板位置，设置为0表示没有刹车
				}
				if (received_data.find("a") != received_data.end()) {
					targetSteering = -1.0f;
				}
				else if (received_data.find("d") != received_data.end()) {
					targetSteering = 1.0f;
				}
				else {
					targetSteering = 0.0f;
				}
				if (currentSteering < targetSteering) {
					currentSteering = std::min(currentSteering + steeringSpeed, targetSteering);
				}
				else if (currentSteering > targetSteering) {
					currentSteering = std::max(currentSteering - steeringSpeed, targetSteering);
				}

				if (received_data.find("g") != received_data.end()) {
					pControl->handbrake = true;  // 手刹状态，设置为false表示手刹未拉起
				}
				if (received_data.find("s") != received_data.end()) {
					pControl->gear = ESimOne_Gear_Mode_Reverse;  // 档位设置，这里将档位设置为1
					tmp = 0.7f;
					pControl->steering = 0.0f;

				}
				pControl->throttle = tmp;
				pControl->steering = currentSteering;

			}
			SimOneAPI::SetDrive(MainVehicleId, pControl.get());
		}

		SimOneAPI::NextFrame(frame);
	}

	// 释放资源
	delete server;
	delete controller;
	return 0;
}
