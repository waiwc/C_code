#pragma once

#include "SimOneServiceAPI.h"
#include <iostream>

using SSD::SimPoint3D;
using SSD::SimString;

int adclanedis(double lanel, double laner)
{
	if (lanel < 2)
		return -1;
	else if (laner < 2)
		return 1;
	else
		return 0;
}

void SampleGetDistanceToLaneBoundary(const SimPoint3D& pos, int* lane)
{
	SimString laneId;
	double distToLeft, distToRight, distToLeft2D, distToRight2D;
	if (!SimOneAPI::GetDistanceToLaneBoundary(pos, laneId, distToLeft, distToRight, distToLeft2D, distToRight2D))
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Error: lane is not found.");
		return;
	}
	*lane = adclanedis(abs(distToLeft), abs(distToRight));

	//SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "lane id: %s", laneId.GetString());
	// SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "[distToLeft, distToRight]: %f, %f", distToLeft, distToRight);
	//SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "[distToLeft2D, distToRight2D]: %f, %f", distToLeft2D, distToRight2D);
}