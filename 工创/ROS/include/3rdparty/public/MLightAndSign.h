#pragma once

#include "libexport.h"
#include "public/common/MSignal.h"
#include "public/common/MObject.h"
#include "SSD/SimString.h"

namespace HDMapStandalone {

	class LIBEXPORT MLightAndSign {
	public:

		//@brief Get id list of traffic lights in the map.
		//@return Returns id list.
		static SSD::SimStringVector GetTrafficLightIds();

		//@brief Get id list of stopLines in the map.
		//@return Returns id list.
		static SSD::SimStringVector GetStoplineIds();

		//@brief Get id list of crosswalks in the map.
		//@return Returns id list.
		static SSD::SimStringVector GetCrosswalkIds();

		//@brief Get traffic lights in the map.
		//@return Returns MSignal list.
		static SSD::SimVector<MSignal> GetTrafficLightList();

		//@brief Get traffic signs in the map.
		//@return Returns sign MSignal list.
		static SSD::SimVector<MSignal> GetTrafficSignList();

		//@brief Get stopLines in the map.
		//@return Returns MObject list.
		static SSD::SimVector<MObject> GetStoplineList();

		//@brief Get crosswalks in the map.
		//@return Returns MObject list.
		static SSD::SimVector<MObject> GetCrosswalkList();

		//@brief Get the list of waiting areas that belongs to traffic light's validity
		//@param light is the specified traffic light.
		//@return Returns waiting area MObject list.
		static SSD::SimVector<MObject> GetWaitingAreas(const MSignal& light);

		//@brief Get the list of waiting areas that belongs to traffic light's validity
		//@param light is the specified traffic light.
		//laneName with this format roadId_sectionIndex_laneId.
		//@return Returns waiting area MObject list.
		static SSD::SimVector<MObject> GetWaitingAreas(const MSignal& light, const SSD::SimString& laneName);

		//@brief Get the list of stoplines that belongs to traffic light's validity
		//@param light is the specified traffic light.
		//laneName with this format roadId_sectionIndex_laneId.
		//@return Returns stopline MObject list.
		static SSD::SimVector<MObject> GetStoplineList(const MSignal& light, const SSD::SimString& laneName);

		//@brief Get the list of crosswalks that belongs to traffic light's validity
		//@param light is the specified traffic light.
		//laneName with this format roadId_sectionIndex_laneId.
		//@return Returns crosswalk MObject list.
		static SSD::SimVector<MObject> GetCrosswalkList(const MSignal& light, const SSD::SimString& laneName);

		//@brief Get the list of traffic lights in the specified lane's road neighborhood.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName must exist in current map.
		//@return Returns traffic light MSignal list.
		static SSD::SimVector<MSignal> GetTrafficLightList(const SSD::SimString& laneName);

		//@brief Get the list of stopLine in the specified lane's road neighborhood.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName must exist in current map.
		//@return Returns crosswalk MObject list.
		static SSD::SimVector<MObject> GetStoplineList(const SSD::SimString& laneName);

		//@brief Get the list of crosswalk in the specified lane's road neighborhood.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName must exist in current map.
		//@return Returns crosswalk MObject list.
		static SSD::SimVector<MObject> GetCrosswalkList(const SSD::SimString& laneName);

		//@brief Get the list of traffic signs in the specified lane's road neighborhood.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName must exist in current map.
		//@return Returns traffic sign MSignal list.
		static SSD::SimVector<MSignal> GetTrafficSignList(const SSD::SimString& laneName);

		//@brief Get the list of traffic signs in the circle of center point.
		//@param center point. radius is the radius of circle.
		//@return Returns traffic sign MSignal list.
		static SSD::SimVector<MSignal> GetTrafficSignList(const SSD::SimPoint3D& center, const double& radius);

		//@brief Get the list of signals in the specified lane's region.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName must exist in current map.
		//@return Returns any type of MSignal list.
		static SSD::SimVector<MSignal> GetSignalListOnLaneByType(const SSD::SimString& laneName, const SSD::SimString& type);

		//@brief Get the list of traffic lights in the specified junction.
		//@param junctionId is junction's id.
		//@pre-condition junction must exist in current map.
		//validityIndexList as an output parameter, which returns list of validity indexes.
		//@return Returns traffic light MSignal list.
		static SSD::SimVector<MSignal> GetTrafficLightList(const long& junctionId, SSD::SimVector<SSD::SimVector<int>>& validityIndexList);

		//@brief Get stopline object by id.
		//@param id is the id of stopline.
		//@pre-condition stopline id must exist in current map.
		//@return Returns stopline object.
		static MObject GetStopline(const long& id);

		//@brief Get traffic light object by id.
		//@param id is the id of traffic light.
		//@pre-condition traffic light id must exist in current map.
		//@return Returns traffic light object.
		static MSignal GetTrafficLight(const long& id);

		//@brief Check whether 2 traffic lights's controlled roads have conflict region.
		//@param id1 is the id of 1st traffic light.
		//id2 is the id of 2nd traffic light.
		//@pre-condition 2 traffic lights must exist in current map.
		//@return Returns true if has conflect region, else returns false.
		static bool HasConflictRegion(const long& id1, const long& id2);
	};
}