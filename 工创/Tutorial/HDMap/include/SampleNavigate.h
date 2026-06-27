#pragma once
#include "SimOneServiceAPI.h"
#include <iostream>
#include "SampleGetRoadLength.h"
#include "SampleIsTwoSideRoad.h"

using SSD::SimString;

SSD::SimVector<long> SampleNavigate(const SSD::SimPoint3DVector& inputPoints)
{
	SSD::SimVector<int> indexOfValidPoints;
	SSD::SimVector<long> roadIdList;
	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "***********************Navigate***********************");
	bool result = SimOneAPI::Navigate(inputPoints, indexOfValidPoints, roadIdList);
	if (!result)
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Not exists!");
	}
	for (auto&item: indexOfValidPoints)
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "indexOfValidPoint: %d", item);
	}
	for (auto&item : roadIdList)
	{
		//26. GetRoadLength
		SampleGetRoadLength(item);
		//27. IsTwoSideRoad
		SampleIsTwoSideRoad(item);

	}
	return roadIdList;
}
