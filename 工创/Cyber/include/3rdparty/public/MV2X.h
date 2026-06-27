#pragma once

#include "libexport.h"
#ifdef STATIC_SSD
#include "SSD/public/SimPoint3D.h"
#include "SSD/public/SimVector.h"
#else
#include "SSD/SimPoint3D.h"
#include "SSD/SimVector.h"
#endif

namespace HDMapStandalone
{
	struct MSignal;
}

namespace HDMapStandalone {

	class LIBEXPORT MV2X {
	public:
		//@brief Get the alert paths for specified traffic sign with the distance of range from backwardMaximum to forwardMaximum.
		//@param sign is the specified traffic sign.
		//forwardMaximum means forward distance at maximum if current lane can extend to.
		//backwardMaximum means backward distance at maximum if current lane can extend to.
		//@return Returns list of paths.
		static SSD::SimVector<SSD::SimPoint3DVector> GetAlertPaths(const MSignal& sign, const double& forwardMaximum, const double& backwardMaximum);

		//@brief Get junction id list of the specified area.
		//@param center point. radius is the radius of circle.
		//@return Returns junction id list.
		static SSD::SimVector<long> GetJunctionList(const SSD::SimPoint3D& center, const double& radius);
	};
}