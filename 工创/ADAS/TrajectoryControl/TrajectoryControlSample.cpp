#include "SimOneServiceAPI.h"
#include "SimOneSensorAPI.h"
#include "SSD/SimPoint3D.h"
#include "UtilDriver.h"
#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>
#include <iomanip> 
#include <thread>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>

// 定义一个结构体来保存轨迹点信息
struct TrajectoryPoint {
	double relative_time;
	double posx;
	double posy;
	double accel;
	double theta;
	double s;
	double speed;
};
size_t currentIndex = 0;  // 初始化时设为0
std::vector<TrajectoryPoint> traj_data; // 动态数组来存储轨迹点数据

void sortTrajectoryByTime(std::vector<TrajectoryPoint>& traj_data) {
	std::sort(traj_data.begin(), traj_data.end(), [](const TrajectoryPoint& a, const TrajectoryPoint& b) {
		return a.relative_time < b.relative_time;
	});
}


// 去除字符串首尾的空白字符
std::string trim(const std::string& str) {
	size_t first = str.find_first_not_of(" \t");
	size_t last = str.find_last_not_of(" \t");
	return str.substr(first, (last - first + 1));
}

// 从文件中读取轨迹点信息
bool loadTrajectoryData(const std::string& filename) {
	std::ifstream infile(filename);
	if (!infile) {
		std::cerr << "Failed to open trajectory file: " << filename << std::endl;
		return false;
	}

	std::string line;
	int lineNumber = 0;
	while (std::getline(infile, line)) {
		if (line.empty()) {
			std::cout << "Skipping empty line: " << lineNumber + 1 << std::endl;
			continue;
		}

		std::istringstream ss(line);
		std::string value;
		TrajectoryPoint p;
		lineNumber++;

		if (line.back() == ',') {
			line.pop_back();
		}

		try {
			if (std::getline(ss, value, ',')) {
				p.relative_time = std::stof(trim(value));
			}
			if (std::getline(ss, value, ',')) {
				p.posx = std::stof(trim(value));
			}
			if (std::getline(ss, value, ',')) {
				p.posy = std::stof(trim(value));
			}
			if (std::getline(ss, value, ',')) {
				p.accel = std::stof(trim(value));
			}
			if (std::getline(ss, value, ',')) {
				p.theta = std::stof(trim(value));
			}
			if (std::getline(ss, value, ',')) {
				p.s = std::stof(trim(value));
			}
			if (std::getline(ss, value, ',')) {
				p.speed = std::stof(trim(value));
			}
		}
		catch (const std::exception& e) {
			std::cerr << "Error parsing line " << lineNumber << ": " << e.what() << std::endl;
			continue;
		}

		traj_data.push_back(p);
	}
	infile.close();

	if (traj_data.empty()) {
		std::cerr << "No data loaded from file: " << filename << std::endl;
		return false;
	}

	return true;
}

// 平滑转向角函数（用简单的低通滤波器平滑转向角）
double smoothSteering(double currentSteering, double previousSteering, double speed, double minAlpha = 0.01f, double maxAlpha = 0.1f) {
	// 根据速度动态调整alpha，速度越快滤波越强
	double alpha = std::max(minAlpha, maxAlpha - speed * 0.01f); // 简单控制alpha变化，速度增大时alpha减小
	return previousSteering * (1.0f - alpha) + currentSteering * alpha;
}

// 计算方向盘转角，加入前视距离和转向角变化限制
double calculateSteering(const std::vector<TrajectoryPoint>& traj_data, const SimOne_Data_Gps* pGps, size_t& currentIndex, double lookAheadDistance = 5.0f) {
	double minDist = std::numeric_limits<double>::max();
	size_t closestIndex = currentIndex;

	// 定义允许的查找范围：前最多 100 个点，后最多 150 个点
	size_t startSearchIndex = (currentIndex >= 100) ? currentIndex - 100 : 0;
	size_t endSearchIndex = std::min(currentIndex + 150, traj_data.size() - 1);

	// 在限定范围内找到最接近的轨迹点
	for (size_t i = startSearchIndex; i <= endSearchIndex; ++i) {
		double dist = std::hypot(pGps->posX - traj_data[i].posx, pGps->posY - traj_data[i].posy);
		if (dist < minDist) {
			minDist = dist;
			closestIndex = i;
		}
	}

	// 更新 currentIndex 为找到的最近点，确保按顺序执行
	currentIndex = closestIndex;

	// 根据前视距离找到目标点
	size_t targetIndex = closestIndex;
	minDist = 0.0f;  // 重置距离计算用于前视目标点
	while (targetIndex < endSearchIndex && minDist < lookAheadDistance) {
		targetIndex++;
		minDist = std::hypot(pGps->posX - traj_data[targetIndex].posx, pGps->posY - traj_data[targetIndex].posy);
	}

	// 计算方向盘角度使用“纯跟踪”方法
	double dx = traj_data[targetIndex].posx - pGps->posX;
	double dy = traj_data[targetIndex].posy - pGps->posY;

	// 计算目标角度并归一化
	double targetHeading = atan2(dy, dx);
	if (targetHeading > M_PI) targetHeading -= 2.0 * M_PI;
	if (targetHeading < -M_PI) targetHeading += 2.0 * M_PI;

	// 获取当前车辆的方向角并归一化
	double currentHeading = pGps->oriZ;
	if (currentHeading > M_PI) currentHeading -= 2.0 * M_PI;
	if (currentHeading < -M_PI) currentHeading += 2.0 * M_PI;

	// 计算角度误差
	double alfa = targetHeading - currentHeading;
	if (alfa > M_PI) alfa -= 2.0 * M_PI;
	if (alfa < -M_PI) alfa += 2.0 * M_PI;

	// 计算前视点到车辆的距离
	double ld = std::hypot(dx, dy);

	// 使用“纯跟踪”算法计算方向盘角度
	double L = 1.3 + 1.55; // 轴距和前视距离的加权值，可以根据实际情况调整
	double steeringAngle = -atan2(2.0 * L * sin(alfa), ld) * 36.0 / (7.0 * M_PI); // 转换为SimOne的方向盘控制角度

	std::cout << "Reference Point: ("
		<< "X: " << traj_data[closestIndex].posx << ", "
		<< "Y: " << traj_data[closestIndex].posy << ", "
		<< "PPPX: " << pGps->posX << ", "
		<< "PPPY: " << pGps->posY << ", "
		<< "dx: " << dx << ", "
		<< "dy: " << dy << ", "
		<< "targetHeading: " << targetHeading << ", "
		<< "steeringAngle: " << steeringAngle << ", "
		<< "currentHeading: " << currentHeading << ", "
		<< "Relative Time: " << traj_data[closestIndex].relative_time << ")\n"
		<< "Current Steering Angle: " << steeringAngle << " degrees\n";

	return steeringAngle;
}



// 根据目标速度和当前速度平滑控制油门
double calculateThrottle(float currentSpeed, float targetSpeed) {
	float speedDifference = targetSpeed - currentSpeed;
	double throttle = 0.0f;

	if (speedDifference > 0) {
		throttle = std::min(0.5f, speedDifference * 0.1f); // 最大油门值设为 0.5，平滑增加
	}
	else {
		throttle = 0.0f;
	}

	return throttle;
}

// 车辆控制函数
void controlVehicle(const char* MainVehicleId, float throttle, float brake, float steering) {
	std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();
	pControl->throttle = throttle;
	pControl->brake = brake;
	pControl->steering = steering;
	pControl->handbrake = false;
	pControl->isManualGear = false;
	pControl->gear = static_cast<ESimOne_Gear_Mode>(1);
	SimOneAPI::SetDrive(MainVehicleId, pControl.get());
}

int main() {
	const char* MainVehicleId = "0";
	SimOneAPI::InitSimOneAPI(MainVehicleId, true);
	SimOneAPI::SetDriverName(MainVehicleId, "Manual");

	if (!loadTrajectoryData("test.csv")) {
		std::cerr << "Failed to load trajectory data." << std::endl;
		return -1;
	}

	std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
	sortTrajectoryByTime(traj_data);
	double previousSteering = 0.0f;

	while (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Running) {
		int frame = SimOneAPI::Wait();

		if (SimOneAPI::GetGps(MainVehicleId, pGps.get())) {
			double steering = calculateSteering(traj_data, pGps.get(), currentIndex, 7.0f); // 设置前视距离为5米
			steering = smoothSteering(steering, previousSteering, 0.07f);      // 平滑处理方向盘角
			previousSteering = steering;

			double currentSpeed = std::sqrt(pGps->velX * pGps->velX + pGps->velY * pGps->velY + pGps->velZ * pGps->velZ);
			size_t closestIndex = 0;
			double minDist = std::numeric_limits<double>::max();
			for (size_t i = 0; i < traj_data.size(); ++i) {
				double dist = std::hypot(pGps->posX - traj_data[i].posx, pGps->posY - traj_data[i].posy);
				if (dist < minDist) {
					minDist = dist;
					closestIndex = i;
				}
			}
			double targetSpeed = traj_data[closestIndex].speed;

			double throttle = calculateThrottle(currentSpeed, targetSpeed);
			double brake = (currentSpeed > targetSpeed + 0.5f) ? 0.1f : 0.0f;

			controlVehicle(MainVehicleId, throttle, brake, steering);

			//std::cout << "Current Speed: " << currentSpeed << " m/s, Target Speed: " << targetSpeed
			//	<< " m/s, Throttle: " << throttle << ", Steering Angle: " << steering << " degrees" << std::endl;
		}

		if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop) {
			break;
		}

		SimOneAPI::NextFrame(frame);
	}

	return 0;
}
