#pragma once
#include "SimOneServiceAPI.h"
#include <iostream>

using SSD::SimString;
using SSD::SimPoint3D;

void SampleGetLaneSampleByLocation(const SSD::SimPoint3D& pos, HDMapStandalone::MLaneInfo& info)
{
	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "*********************GetLaneSampleByLocation*********************");
	bool result = SimOneAPI::GetLaneSampleByLocation(pos, info);
	if (!result)
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "*********************Not get Lane info by mainVehicle location !!!*********************");
	}
	else {
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "laneName:%s", info.laneName.GetString());
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "leftBoundary size :%d", info.leftBoundary.size());
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "rightBoundary size :%d", info.rightBoundary.size());
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "centerLine size :%d", info.centerLine.size());
	}
}
