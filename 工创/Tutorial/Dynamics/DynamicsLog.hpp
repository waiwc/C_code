#pragma once

#include <fstream>
#include <iostream>
#include <unordered_map>
#include "UtilMath.hpp"

class DynamicsLog
{
public:
	DynamicsLog() {}
	~DynamicsLog()
	{
		mFile.close();
	}

	void addData(const std::string& name, const float data)
	{
		if (mDataMap.find(name) != mDataMap.end())
		{
			mDataMap[name].push_back(data);
		}
		else
		{
			std::cout << name << " not found" << std::endl;
		}
	}
	void setTitle() {
		std::vector<double> data;
		std::vector<std::string> titleRow =
		{
			"Time_s",
			"ForwardSpeed_km_h",
			"SteeringInput",
			"ThrottleInput",
			"BrakeInput",
			"EngineRotationSpeed",
			"Pos_X",
			"Pos_Y",
			"Pos_Z",
			"AccX",
			"AccY",
			"AccZ",
			"AngularVelX",
			"AngularVelY",
			"AngularVelZ",
			"Rot0_deg",
			"Rot1_deg",
			"Rot2_deg",
			"Ax_m_s2",
			"Ay_m_s2",
			"wheelSpeedFL",
			"wheelSpeedFR",
			"wheelSpeedRL",
			"wheelSpeedRR"
		};
		for (size_t i = 0; i < titleRow.size(); ++i)
		{
			mDataMap[titleRow[i]] = data;
		}
	}

	void outputDataToFile(const std::string& path)
	{
		std::ofstream myFile;
		myFile.open(path);
		for (auto& c : mDataMap)
		{
			myFile << c.first << ",";
		}
		myFile << std::endl;

		size_t dataSize = mDataMap.begin()->second.size();
		for (size_t i = 0; i < dataSize; ++i)
		{
			for (auto& c : mDataMap)
			{
				myFile << c.second[i] << ",";
			}
			myFile << std::endl;
		}
	}

	void saveData(SimOne_Data_Gps *gpsPtr, SimOne_Data_Vehicle_Extra *pVehExtraState) {
		addData("Time_s", pVehExtraState->extra_states[0]);
		addData("ForwardSpeed_km_h", UtilMath::mpsToKmph(UtilMath::calculateSpeed(gpsPtr->velX, gpsPtr->velY, gpsPtr->velZ)));
		addData("SteeringInput", gpsPtr->steering);
		addData("ThrottleInput", gpsPtr->throttle);
		addData("BrakeInput", gpsPtr->brake);
		addData("EngineRotationSpeed", gpsPtr->engineRpm);
		addData("Pos_X", gpsPtr->posX);
		addData("Pos_Y", gpsPtr->posY);
		addData("Pos_Z", gpsPtr->posZ);
		addData("AccX", gpsPtr->accelX);
		addData("AccY", gpsPtr->accelY);
		addData("AccZ", gpsPtr->accelZ);
		addData("AngularVelX", gpsPtr->angVelX);
		addData("AngularVelY", gpsPtr->angVelY);
		addData("AngularVelZ", gpsPtr->angVelZ);
		addData("Rot0_deg", gpsPtr->oriX);
		addData("Rot1_deg", gpsPtr->oriY);
		addData("Rot2_deg", gpsPtr->oriZ);
		addData("Ax_m_s2", gpsPtr->accelX *std::cos(gpsPtr->oriZ) + gpsPtr->accelY *std::sin(gpsPtr->oriZ));
		addData("Ay_m_s2", -gpsPtr->accelX*std::sin(gpsPtr->oriZ) + gpsPtr->accelY *std::cos(gpsPtr->oriZ));
		addData("wheelSpeedFL", gpsPtr->wheelSpeedFL);
		addData("wheelSpeedFR", gpsPtr->wheelSpeedFR);
		addData("wheelSpeedRL", gpsPtr->wheelSpeedRL);
		addData("wheelSpeedRR", gpsPtr->wheelSpeedRR);
	}

private:
	std::ofstream mFile;
	std::unordered_map<std::string, std::vector<double>> mDataMap;
};
