#pragma once

#include "SimOneServiceAPI.h"
#include <iostream>

using SSD::SimString;
using SSD::SimStringVector;

void SampleGetSectionLaneList(const SimString& laneId, SSD::SimStringVector& sectionLaneList)
{
	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "*********************GetSectionLaneList*********************");
	if (!SimOneAPI::GetSectionLaneList(laneId, sectionLaneList))
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "*********************GetSectionLaneList return false !!!*********************");
		return;
	}

	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "*********************GetSectionLaneList sectionLaneList: %d*********************", sectionLaneList.size());
	for (int i = 0; i < sectionLaneList.size(); i++) {
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "*********************GetSectionLaneList sectionLaneList id: %s*********************", sectionLaneList[i].GetString());
	}
}