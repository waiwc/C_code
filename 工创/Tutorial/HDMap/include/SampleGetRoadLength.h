#pragma once

#include "SimOneServiceAPI.h"
#include <iostream>


void SampleGetRoadLength(const long& roadId)
{
	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "*********************GetRoadLength*********************");
	double roadLength = SimOneAPI::GetRoadLength(roadId);
	printf("roadId:%ld, length---------:%lf\n,", roadId, roadLength);
	//SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "*********************GetRoadLength length:%lf*********************", roadLength);
}
