#pragma once
#include "SimOneServiceAPI.h"
#include <iostream>

using SSD::SimString;

void SampleGetLaneLength(const SSD::SimString& id)
{
	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "*********************GetLaneLength*********************");
	
	double LaneLength = SimOneAPI::GetLaneLength(id);
	
	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "*********************LaneLength is %lf*********************", LaneLength);
}
