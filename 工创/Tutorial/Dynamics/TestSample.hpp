#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <algorithm>
#include "SimOnePNCAPI.h"
#include "SimOneSensorAPI.h"
#include "DynamicsLog.hpp"
#include "UtilMath.hpp"
#include <thread>

#ifndef PI
#define  PI   3.14159265358979323846
#endif

class TestSample
{
public:
	TestSample() {
		mDynaLog.setTitle();
	}
	~TestSample() {}
	bool DriveToSpeed(const char* mainVehicleId, float targetSpeed, float throttle, std::string path)
	{
		std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique <SimOne_Data_Control>();
		std::unique_ptr<SimOne_Data_Gps> gpsPtr = std::make_unique<SimOne_Data_Gps>();
		bool speedReached = false;
		ESimOne_Data_Vehicle_State simOneState = ESimOne_Data_Vehicle_State_SO_TimePassed;
		std::unique_ptr<SimOne_Data_Vehicle_Extra> pVehExtraState = std::make_unique<SimOne_Data_Vehicle_Extra>();
		while (1) {
			SimOneAPI::GetGps(mainVehicleId, gpsPtr.get());
			pControl->throttle = (float)throttle;
			pControl->brake = 0.0f;
			pControl->steering = 0.0f;
			pControl->handbrake = false;
			pControl->isManualGear = false;
			pControl->gear = ESimOne_Gear_Mode_Drive;
			pControl->timestamp = gpsPtr->timestamp;
			bool testre = SimOneAPI::SetDrive(mainVehicleId, pControl.get());

			SimOneAPI::RegisterVehicleState("0", &simOneState, 1);
			SimOneAPI::GetVehicleState("0", pVehExtraState.get());
			mDynaLog.saveData(gpsPtr.get(), pVehExtraState.get());
			if (UtilMath::mpsToKmph(UtilMath::calculateSpeed(gpsPtr->velX, gpsPtr->velY, gpsPtr->velZ)) >= targetSpeed) {
				speedReached = true;
				mDynaLog.outputDataToFile(path);
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		if (throttle > 0.99f || speedReached) {
			return speedReached;
		}
		else {
			return DriveToSpeed(mainVehicleId, targetSpeed, throttle + 0.3f > 1.f ? 1.f : throttle + 0.3f, path);
		}
	}

	void FullBrakeTest(const char* mainVehicleId, float brake, std::string path)
	{
		std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique <SimOne_Data_Control>();
		std::unique_ptr<SimOne_Data_Gps> gpsPtr = std::make_unique<SimOne_Data_Gps>();

		DriveToSpeed(mainVehicleId, 100.f, 1.0f, path);

		ESimOne_Data_Vehicle_State simOneState = ESimOne_Data_Vehicle_State_SO_TimePassed;
		std::unique_ptr<SimOne_Data_Vehicle_Extra> pVehExtraState = std::make_unique<SimOne_Data_Vehicle_Extra>();
		bool stopped = false;
		pControl->throttle = 0.0f;
		pControl->brake = brake;
		pControl->steering = 0.0f;
		pControl->handbrake = false;
		pControl->isManualGear = false;
		pControl->gear = ESimOne_Gear_Mode_Drive;
		while (1) {
			SimOneAPI::SetDrive(mainVehicleId, pControl.get());
			if (SimOneAPI::GetGps(mainVehicleId, gpsPtr.get())) {
				mDynaLog.saveData(gpsPtr.get(), pVehExtraState.get());
				if (UtilMath::mpsToKmph(UtilMath::calculateSpeed(gpsPtr->velX, gpsPtr->velY, gpsPtr->velZ)) < 0.01f) {
					stopped = true;
					mDynaLog.outputDataToFile(path);
					break;
				}
			}
			else
			{
				std::cout << "Fetch GPS failed" << std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}

	enum SteerType
	{
		sineSteer,
		rampSteer,
		stepSteer
	};
	void SteerDynamicTest(const char* mainVehicleId, float targetSpd, SteerType steerType, std::string path)
	{
		float steer = 0.0f;
		float maxSteerDeg = 540.f;
		mStepSteerCurve.addPair(0.0f, 0.0f);
		mStepSteerCurve.addPair(0.0f, 0.0f);
		mStepSteerCurve.addPair(0.01f, 0.00455f);
		mStepSteerCurve.addPair(0.02f, 0.0493f);
		mStepSteerCurve.addPair(0.03f, 0.176f);
		mStepSteerCurve.addPair(0.04f, 0.416f);
		mStepSteerCurve.addPair(0.05f, 0.794f);
		mStepSteerCurve.addPair(0.06f, 1.32f);
		mStepSteerCurve.addPair(0.07f, 2.01f);
		mStepSteerCurve.addPair(0.08f, 2.86f);
		mStepSteerCurve.addPair(0.09f, 3.86f);
		mStepSteerCurve.addPair(0.10f, 5.0f);
		mStepSteerCurve.addPair(0.11f, 6.27f);
		mStepSteerCurve.addPair(0.12f, 7.65f);
		mStepSteerCurve.addPair(0.13f, 9.11f);
		mStepSteerCurve.addPair(0.14f, 10.6f);
		mStepSteerCurve.addPair(0.15f, 12.2f);
		mStepSteerCurve.addPair(0.16f, 13.7f);
		mStepSteerCurve.addPair(0.17f, 15.3f);
		mStepSteerCurve.addPair(0.18f, 16.8f);
		mStepSteerCurve.addPair(0.19f, 18.2f);
		mStepSteerCurve.addPair(0.20f, 19.5f);
		mStepSteerCurve.addPair(0.21f, 20.7f);
		mStepSteerCurve.addPair(0.22f, 21.8f);
		mStepSteerCurve.addPair(0.23f, 22.7f);
		mStepSteerCurve.addPair(0.24f, 23.4f);
		mStepSteerCurve.addPair(0.25f, 24.0f);
		mStepSteerCurve.addPair(0.26f, 24.4f);
		mStepSteerCurve.addPair(0.27f, 24.7f);
		mStepSteerCurve.addPair(0.28f, 24.9f);
		mStepSteerCurve.addPair(0.29f, 25.0f);
		mStepSteerCurve.addPair(0.30f, 25.0f);
		mStepSteerCurve.addPair(30.0f, 25.0f);
		DriveToSpeed(mainVehicleId, targetSpd, 0.3f, path);
		std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique <SimOne_Data_Control>();
		std::unique_ptr<SimOne_Data_Gps> gpsPtr = std::make_unique<SimOne_Data_Gps>();
		ESimOne_Data_Vehicle_State simOneState = ESimOne_Data_Vehicle_State_SO_TimePassed;
		std::unique_ptr<SimOne_Data_Vehicle_Extra> pVehExtraState = std::make_unique<SimOne_Data_Vehicle_Extra>();
		SimOneAPI::RegisterVehicleState("0", &simOneState, 1);
		SimOneAPI::GetVehicleState("0", pVehExtraState.get());
		const float startTime = pVehExtraState->extra_states[0];

		pControl->throttle = 0.0f;
		pControl->brake = 0.0;
		pControl->handbrake = false;
		pControl->isManualGear = false;
		pControl->gear = ESimOne_Gear_Mode_Drive;
		while (1)
		{
			SimOneAPI::GetGps(mainVehicleId, gpsPtr.get());
			SimOneAPI::RegisterVehicleState("0", &simOneState, 1);
			SimOneAPI::GetVehicleState("0", pVehExtraState.get());
			pControl->throttle = speedControl(gpsPtr.get(), UtilMath::kmphToMps(targetSpd));
			if (steerType == sineSteer) {
				pControl->steering =25.0f * sinf(2 * PI* 0.1* (pVehExtraState->extra_states[0] - startTime)*(pVehExtraState->extra_states[0] - startTime))/maxSteerDeg;
				SimOneAPI::SetDrive(mainVehicleId, pControl.get());
				if ((pVehExtraState->extra_states[0] - startTime) >= 10.0f) {
					pControl->steering = 0.0f;
					mDynaLog.outputDataToFile(path);
					break;
				}
			}
			else if (steerType == rampSteer) {
				pControl->steering = 0.8 * (pVehExtraState->extra_states[0] - startTime);
					if (pControl->steering >= 0.5f) {
						pControl->steering = 0.5f;
					}
				SimOneAPI::SetDrive(mainVehicleId, pControl.get());
					if (pVehExtraState->extra_states[0] - startTime >= 25.0f) {
						mDynaLog.outputDataToFile(path);
							break;
					}
			}
			else if (steerType == stepSteer) {
				pControl->steering = mStepSteerCurve.getYVal(pVehExtraState->extra_states[0] - startTime)/maxSteerDeg;
				SimOneAPI::SetDrive(mainVehicleId, pControl.get());
				if (pVehExtraState->extra_states[0] - startTime >= 25.0f) {
					mDynaLog.outputDataToFile(path);
					break;
				}
			}
			mDynaLog.saveData(gpsPtr.get(), pVehExtraState.get());
		}
	}
	
	float speedControl(SimOne_Data_Gps *gpsPtr, float targetSpeed)
	{
		float p = 0.2;
		float throttle = 0.0f;
		double mainVehicleSpeed = UtilMath::calculateSpeed(gpsPtr->velX, gpsPtr->velY, gpsPtr->velZ);
		while (1) {
			return throttle = (targetSpeed - mainVehicleSpeed) * p;
		}
	}

private:
	DynamicsLog mDynaLog;
	LookupTable2D mStepSteerCurve;
};