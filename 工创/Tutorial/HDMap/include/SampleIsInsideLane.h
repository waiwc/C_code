#pragma once
#include "SimOneServiceAPI.h"
#include <iostream>

using SSD::SimString;

void SampleIsInsideLane(const SSD::SimPoint3D& inputPt, const SSD::SimString& laneName, HDMapStandalone::MSideState& sideState)
{
	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "*********************IsInsideLane*********************");
	bool result = SimOneAPI::IsInsideLane(inputPt, laneName, sideState);
	if (!result)
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "*********************inputPoint not inside lane !!!*********************");
	}
	else {
		std::string SideType;
		switch (sideState) {
		case MSideState::eInner:
		{
			SideType = "MSideState::eInner";
		}
		break;
		case MSideState::eLeftSide:
		{
			SideType = "MSideState::eLeftSide";
		}
		break;
		case MSideState::eRightSide:
		{
			SideType = "MSideState::eLeftSide";
		}
		break;
		default:
			SideType = "MSideState::none";
		}
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "inputPoint is inside lane,%s", SideType.c_str());
	}
	
}
