#pragma once

#include "libexport.h"
#include "common/MSignal.h"

namespace HDMapStandalone
{
	class MHDMapTS;
}

namespace HDMapStandalone {

	class LIBEXPORT MV2XTS
	{
	public:

		//@brief Constructor.
		//@param hdmap_ represents reference to class MHDMapTS object.
		MV2XTS(const MHDMapTS& hdmap_);

		//@brief Get the alert paths for specified traffic sign with the distance of range from backwardMaximum to forwardMaximum.
		//@param sign is the specified traffic sign.
		//forwardMaximum means forward distance at maximum if current lane can extend to.
		//backwardMaximum means backward distance at maximum if current lane can extend to.
		//@return Returns list of paths.
		SSD::SimVector<SSD::SimPoint3DVector> GetAlertPaths(const MSignal& sign, const double& forwardMaximum, const double& backwardMaximum) const;

		//@brief Get junction id list of the specified area.
		//@param center point. radius is the radius of circle.
		//@return Returns junction id list.
		SSD::SimVector<long> GetJunctionList(const SSD::SimPoint3D& center, const double& radius) const;

	private:
		const MHDMapTS& hdmap;
	};
}