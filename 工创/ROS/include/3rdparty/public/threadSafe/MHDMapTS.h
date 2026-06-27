#pragma once

#include "libexport.h"
#include "public/common/MEnum.h"
#include "public/threadSafe/MMapTS.h"
#include "public/threadSafe/MLocationTS.h"
#include "public/threadSafe/MLightAndSignTS.h"
#include "public/threadSafe/MRoutingTS.h"
#include "public/threadSafe/MNavigationTS.h"
#include "public/threadSafe/MV2XTS.h"

namespace HDMapStandalone {

	// In this system, laneName is organized as string(SSD::SimString) with this format roadId_sectionIndex_laneId.
	// In public/common/MLaneId.h, MLaneId class is defined to hold this format laneName as data struct.
	// Its method ToString() can convert MLaneId object into roadId_sectionIndex_laneId format string.

	class LIBEXPORT MHDMapTS
	{
	public:

		//@brief Constructor. It will load xodr data immediately. Map data can from  xodr file or xodr content.
		//map data's geographic info must be localENU.
		//If geographic info is not localENU, need to use Wet.exe or WorldEditor to convert xodr into our localENU format.
		//@param xodrFileOrContent represents the xodr file's pathname or xodr content.
		//sourceType represents XodrSourcType for loading data.
		MHDMapTS(const SSD::SimString& xodrFileOrContent, MXodrSourceType sourceType);

		//@brief Constructor. It will load xodr data from database.
		//map data's geographic info must be localENU.
		//If geographic info is not localENU, need to use Wet.exe or WorldEditor to convert xodr into our localENU format.
		//@param dbContent represents the json formatted content string.
		//headerXml represents the header info in xml format.
		MHDMapTS(const SSD::SimString& dbContent, const SSD::SimString& headerXml);

		//@brief Destructor.
		~MHDMapTS();

		//@brief Check whether xodr map is loaded. If failed, return load error code.
		//@param is as an output parameter, which returns load error code.
		//@return Returns true if already loaded with success, false if failed.
		bool IsMapLoaded(MLoadErrorCode& errorCode) const;

		//@brief Get MMapTS class object, so as to access more api.
		//@return Returns reference to MMapTS object.
		const MMapTS& GetMap() const;

		//@brief Get MLocationTS class object, so as to access more api.
		//@return Returns reference to MLocationTS object.
		const MLocationTS& GetLocation() const;

		//@brief Get MLightAndSignTS class object, so as to access more api.
		//@return Returns reference to MLightAndSignTS object.
		const MLightAndSignTS& GetLightAndSign() const;

		//@brief Get MRoutingTS class object, so as to access more api.
		//@return Returns reference to MRoutingTS object.
		const MRoutingTS& GetRouting() const;

		//@brief Get MNavigationTS class object, so as to access more api.
		//@return Returns reference to MNavigationTS object.
		const MNavigationTS& GetNavigation() const;

	private:
		void* mImp = nullptr;
		MMapTS mMap;
		MLocationTS mLocation;
		MLightAndSignTS mLightAndSign;
		MRoutingTS mRouting;
		MNavigationTS mNavigation;
		MV2XTS mV2X;
		bool mbLoaded = false;
		MLoadErrorCode mError;

		friend class MMapTS;
		friend class MLocationTS;
		friend class MLightAndSignTS;
		friend class MRoutingTS;
		friend class MNavigationTS;
		friend class MV2XTS;
	};
}