#pragma once
#include "SimOneServiceAPI.h"
#include <iostream>

void SampleIsTwoSideRoad(const long& roadId)
{
	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "*********************IsTwoSideRoad*********************");
	bool result = SimOneAPI::IsTwoSideRoad(roadId);
	if (!result)
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "*********************road is not twoSide !!!*********************");
	}
	else {
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "*********************road is twoSide*********************");
	}
	
}
