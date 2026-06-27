#pragma once

#include "SimOneServiceAPI.h"
#include <iostream>

using SSD::SimString;
using SSD::SimPoint3D;

void SampleGetJunctionList(const SimString& laneId)
{
	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "*********************GetJunctionList*********************");
	SSD::SimVector<long> JunctionList = SimOneAPI::GetJunctionList();
	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "*********************GetJunctionList size:%d*********************", JunctionList.size());
	for (auto&item : JunctionList)
	{
		//31. IsInJunction
		SampleIsInJunction(laneId, item);
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "JunctionId-----------------: %ld", item);
	}
}