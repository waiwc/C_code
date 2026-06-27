#pragma once

#include "SimOneServiceAPI.h"
#include <iostream>


void SampleGetLaneData(SSD::SimVector<HDMapStandalone::MLaneInfo>& data)
{
	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "*********************GetLaneData*********************");
	SimOneAPI::GetLaneData(data);
	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "*********************GetLaneData size:%d*********************",data.size());
	for (int i = 0; i < data.size(); i++) {
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "*********************GetLaneData laneName:%s*********************", data[i].laneName.GetString());
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "*********************GetLaneData centerLine size:%d*********************", data[i].centerLine.size());
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "*********************GetLaneData leftBoundary size:%d*********************", data[i].leftBoundary.size());
	}
}

