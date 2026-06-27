#pragma once

#include "libexport.h"
#include "public/common/MLaneInfo.h"
#include "public/common/MLaneLink.h"
#include "public/common/MEnum.h"
#include "public/common/MLaneId.h"
#include "public/common/MAbstractRoadInfo.h"
#include "public/common/MObject.h"
#include "public/common/MParkingSpace.h"
#include "public/common/MModelObject.h"

namespace HDMapStandalone
{
	class MHDMapTS;
}

namespace HDMapStandalone {

	class LIBEXPORT MMapTS
	{
	public:

		//@brief Constructor.
		//@param hdmap_ represents reference to class MHDMapTS object.
		MMapTS(const MHDMapTS& hdmap_);

		// 1. Map data query related
		//
		//@brief Get MAbstractRoadInfo object list based on the loaded map.
		//@return Returns MAbstractRoadInfo object list.
		SSD::SimVector<MAbstractRoadInfo> GetAbstractRoadData() const;


		// 2. Lane related
		//
		//@brief Check whether lane exists in current map.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@return Returns true if exists, else returns false.
		bool ContainsLane(const SSD::SimString& laneName) const;

		//@brief Get all lane data in the loaded map.
		//@return Returns MLaneVector for all lane data.
		SSD::SimVector<MLaneInfo> GetLaneData() const;

		//@brief Get all lane data indicated by lane list.
		//@param laneList specifies a list of lane name, with each lane name with this format roadId_sectionIndex_laneId.
		//@return Returns MLaneVector for specified lanes.
		SSD::SimVector<MLaneInfo> GetLaneData(const SSD::SimStringVector& laneList) const;

		//@brief Get all lanes' lane line info in the loaded map.
		//@return Returns MLaneLineInfo list for all lanes.
		SSD::SimVector<MLaneLineInfo> GetLaneLineInfo() const;

		//@brief Get all junction id in the loaded map.
		//@return Returns junction id list.
		SSD::SimVector<long> GetJunctionList() const;

		//@brief Get lane id list for specified road.
		//@param roadId.
		//@return Returns lane id list.
		SSD::SimStringVector GetLaneList(const long& roadId) const;

		//@brief Get lane id list in the same section for specified lane id.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName roadId_sectionIndex_laneId's laneId should not be set as 0, as it does not make sense to use 0.
		//@return Returns lane id list that belong to the same section of specified lane id.
		SSD::SimStringVector GetSectionLaneList(const SSD::SimString& laneId) const;

		//@brief Get lane data of specified lane.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName must exist in current map.
		//@return Returns MLaneInfo object.
		MLaneInfo GetLaneSample(const SSD::SimString& laneName) const;

		//@brief Get all lane data by specified distance in forward direction of current position
		//@param inputPt is a 3d point.
		//laneName with this format roadId_sectionIndex_laneId.
		//forward is the distance in forward direction. Its maximum limit is 200 meters, and it minimum limit is greater than 0 meter.
		//@pre-condition laneName must exist in current map.
		//@return Returns MLaneInfo object list.
		SSD::SimVector<MLaneInfo> GetLaneSample(const SSD::SimPoint3D& inputPt, const SSD::SimString& laneName, const double& forward) const;

		//@brief Get lane sample for specified lane and forward and backward range of current position
		//@param inputPt is a 3d point.
		//laneName with this format roadId_sectionIndex_laneId.
		//forward is the distance in forward direction. Its maximum limit is 200 meters, and it minimum limit is greater than 0 meter.
		//info as an output parameter, is a MLaneInfo object.
		//@pre-condition laneName must exist in current map.
		//@return Returns true if inputPt is valid to define the range of lane sample.
		bool GetCurrentLaneSample(const SSD::SimPoint3D& inputPt, const SSD::SimString& laneName,
			const double& forwardMaximum, const double& backwardMaximum, MLaneInfo& info) const;

		//@brief Get lane's MLaneLink based on lane's name.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName must exist in current map. laneName roadId_sectionIndex_laneId's laneId should not be set as 0,
		//as it does not make sense to use 0.
		//@return Returns MLaneLink object.
		MLaneLink GetLaneLink(const SSD::SimString& laneName) const;

		//@brief Get lane's speed limit.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName must exist in current map.
		//@return Returns value of speed limit.
		double GetLaneSpeedLimit(const SSD::SimString& laneName) const;

		//@brief Get lane's type.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName must exist in current map.
		//@return Returns MLaneType enum value.
		MLaneType GetLaneType(const SSD::SimString& laneName) const;

		//@brief Get length of lane's middle line.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName must exist in current map.
		//@return Returns lane's length.
		double GetLaneLength(const SSD::SimString& laneName) const;

		//@brief Get lane's width in bitangent direction of specified MLaneId.
		//@param laneId is lane's id in MLaneId format.
		//s is the s-value in  s-t coordinate system.
		//@pre-condition laneId must exist in current map.
		//@return Returns lane width by s.
		double GetLaneWidth(const HDMapStandalone::MLaneId& laneId, const double& s) const;

		SSD::SimPoint3D GetLaneDirection(const SSD::SimString& laneName, const double& s) const;

		double GetLaneT(const HDMapStandalone::MLaneId& laneId, const double& roadS) const;
		double GetLaneWidthByRoadS(const HDMapStandalone::MLaneId& laneId, const double& roadS) const;

		//@brief Get length of road.
		//@param roadId is road's id.
		//@pre-condition specified road must exist in current map.
		//@return Returns road's length.
		double GetRoadLength(const long& roadId) const;

		//@brief Check whether specified road is two-side road or not.
		//@param roadId is road's id.
		//@pre-condition specified road must exist in current map.
		//@return Returns true if specified road is two-side road, else returns false.
		bool IsTwoSideRoad(const long& roadId) const;

		//@brief Check whether current lane is driving.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName must exist in current map.
		//@return Returns true if is driving, else returns false.
		bool IsDriving(const SSD::SimString& laneName) const;

		//@brief Check whether current lane is driving and no dirving lane on left side or right side.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName must exist in current map.
		//@return Returns true if the lane is driving with no dirving lane on left side or right, else returns false.
		bool IsDrivingAtEdge(const SSD::SimString& laneName) const;

		//@brief Get the list of cross hatch in the map.
		//@return Returns cross hatch MObject list.
		SSD::SimVector<MObject> GetCrossHatchList() const;

		//@brief Get the list of cross hatch in the specified lane's road neighborhood.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//@pre-condition laneName must exist in current map.
		//@return Returns cross hatch MObject list.
		SSD::SimVector<MObject> GetCrossHatchList(const SSD::SimString& laneName) const;

		//@brief Check whether current lane belongs to a junction.
		//@param laneName with this format roadId_sectionIndex_laneId.
		//juncId is as an output parameter, which returns the owner junction id.
		//@pre-condition laneName must exist in current map.
		//@return Returns true if the lane belongs to a junction, else returns false.
		bool IsInJunction(const SSD::SimString& laneName, long& juncId) const;

		//@brief Get center point of the specified junction.
		//@param junctionId is junction's id.
		//@pre-condition junction must exist in current map.
		//@return Returns center point.
		SSD::SimPoint3D GetJunctionCenter(const long& junctionId) const;


		//3. Location query related: go to MLocation.h
		//


		//4. Traffic light and sign related: go to MLightAndSign.h
		//


		//5. Parking space related
		//
		//@brief Get id list of parkingSpaces in the map.
		//@return Returns id list.
		SSD::SimStringVector GetParkingSpaceIds() const;

		//@brief Get parkingSpace list in the map.
		//@return Returns MParkingSpace object list.
		SSD::SimVector<MParkingSpace> GetParkingSpaceList() const;

		//@brief Get id list of parkingSpaces regarding of a position and a distance range any parkingSpace is close to.
		//@param inputPt is the referenced position; distance means the distance range.
		//ids is an output parameter to specify the ids of parkingSpaces.
		//@return Returns true if any parkingSpace is found, else returns false.
		bool GetParkingSpaceIds(const SSD::SimPoint3D& inputPt, double distance, SSD::SimStringVector& ids) const;

		//@brief Check whether parkingSpace exists in current map.
		//@param id is the id of parkingSpace.
		//@return Returns true if exists, else returns false.
		bool ContainsParkingSpace(const SSD::SimString& id) const;


		//6. Model related
		//
		//@brief Get model object list by specfiying object type list in the map.
		//@param typeList is the type list, string format as "type1,type2".
		//@return Returns model object list.
		SSD::SimVector<MModelObject> GetModelObject(const SSD::SimString& typeList) const;
	private:
		const MHDMapTS& hdmap;
	};
}