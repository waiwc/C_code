#pragma once
#include "SimOneServiceAPI.h"
#include <iostream>

using SSD::SimString;

void SampleIsInJunction(const SSD::SimString& id, long& juncId)
{
	SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "*********************IsInJunction*********************");
	
	bool result = SimOneAPI::IsInJunction(id, juncId);
	if (!result)
	{
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "*********************Is not In Junction !!!*********************");
	}
	else {
		SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "*********************juncId %s*********************",juncId);
	}
}
