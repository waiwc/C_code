#pragma once

#include "libexport.h"
#include "public/common/MRouteElement.h"
#include "public/common/MRoutePath.h"
#include "public/common/MAbstractRoadInfo.h"

namespace HDMapStandalone
{
	class MHDMapTS;
}

namespace HDMapStandalone {

	class LIBEXPORT MRoutingTS
	{
	public:

		//@brief Constructor.
		//@param hdmap_ represents reference to class MHDMapTS object.
		MRoutingTS(const MHDMapTS& hdmap_);

		//@brief Validate if the input point is valid on or close to a driving lane.
		//@param [x, y] represents a 2d point.
		//@return Returns true if valid, false if not.
		bool ValidatePoint(const double& x, const double& y) const;

		//@brief Validate if the input point is valid on or close to a driving lane.
		//@param pt represents a 3d point.
		//laneName as an output parameter that returns laneName that input pt is inside.
		//s as an output parameter that returns s value in [s, t] coordinate system.
		//@return Returns true if valid, false if not.
		bool ValidatePoint(const SSD::SimPoint3D& pt, SSD::SimString& laneName, double& s) const;

		//@brief Get waypoint size of MRouting instance.
		//@return Returns waypoint size.
		int GetPointSize() const;

		//@brief Generate routing organized from topology nodes.
		//@param Output ASCII file that accepts generated routing topo data(proto data as original)
		//@return Returns list of MRoute object. Returns nullptr if result is empty.
		bool GenerateRouteTopo(SSD::SimVector<MRouteElement>& routeTopo, const char* outputName = nullptr) const;

		//@brief Generate routing organized as a smooth path. The path is a fitted curve based on the result of GenerateRouteTopo().
		//Waypoint list are prebuilt for MRouting instance, so no parameters for input waypoint here.
		//path is an output parameter to return MRoutePath.
		//@return true if success, false if not.
		bool GenerateRoute(MRoutePath& path) const;

		//@brief Generate routing organized as a smooth path. The path is a fitted curve based on the result of GenerateRouteTopo().
		//Waypoint list are prebuilt for MRouting instance, so no parameters for input waypoint here.
		//path is an output parameter to return MRoutePath.
		//routes is an output parameter to return list of MRouteElement that defines each route.
		//@return true if success, false if not.
		bool GenerateRoute(MRoutePath& path, SSD::SimVector<MRouteElement>& routes) const;

		//@brief Generate routing organized as a smooth path, only when all input sample nodes are valid on driving lane.
		//The path is a fitted curve based on the result of GenerateRouteTopo().
		//@param sampleNodes is waypoint list as input to generate routing.
		//indexOfValidPoints is an output parameter to return the valid points' index regarding to the input vector sampleNodes,
		//so that API user would know about the invalid sample nodes.
		//path is an output parameter to return MRoutePath.
		//@return true if success, false if not.
		bool GenerateRoute(const SSD::SimPoint3DVector& sampleNodes, SSD::SimVector<int>& indexOfValidPoints, MRoutePath& path) const;

		//@brief Generate routing organized as a smooth path.
		//It respects sampleNodes' height value when considering sample node's validity.
		//The path is a fitted curve based on the result of GenerateRouteTopo().
		//@param sampleNodes is waypoint list as input to generate routing.
		//indexOfValidPoints is an output parameter to return the valid points' index regarding to the input vector sampleNodes,
		//so that API user would know about the invalid sample nodes.
		//path is an output parameter to return MRoutePath.
		//@return true if success, false if not.
		bool GenerateRouteWithHeight(const SSD::SimPoint3DVector& sampleNodes, SSD::SimVector<int>& indexOfValidPoints, MRoutePath& path) const;

		//@brief Generate routing organized as a smooth path.
		//The path is a fitted curve based on the result of GenerateRouteTopo().
		//@param sampleNodes is waypoint list as input to generate routing.
		//indexOfValidPoints is an output parameter to return the valid points' index regarding to the input vector sampleNodes,
		//so that API user would know about the invalid sample nodes.
		//path is an output parameter to return MRoutePath.
		//routePtList is an output parameter to return MRoutePoint list.
		//@return true if success, false if not.
		bool GenerateRoute(const SSD::SimPoint3DVector& sampleNodes, SSD::SimVector<int>& indexOfValidPoints,
			MRoutePath& path, SSD::SimVector<MRoutePoint>& routePtList) const;

		//@brief Generate routing organized as a smooth path. It will pick one as best if there are multiple possibilities for input sample nodes.
		//It respects sampleNodes' height value when considering sample node's validity.
		//The path is a fitted curve based on the result of GenerateRouteTopo().
		//@param sampleNodes is waypoint list as input to generate routing.
		//indexOfValidPoints is an output parameter to return the valid points' index regarding to the input vector sampleNodes,
		//so that API user would know about the invalid sample nodes.
		//path is an output parameter to return MRoutePath.
		//routePtList is an output parameter to return MRoutePoint list.
		//@return true if success, false if not.
		bool GenerateBestRoute(const SSD::SimPoint3DVector& sampleNodes, SSD::SimVector<int>& indexOfValidPoints,
			MRoutePath& path, SSD::SimVector<MRoutePoint>& routePtList);

		//@brief Generate routing organized as a smooth path based on input MAbstractRoadInfoVector.
		//Waypoint list are prebuilt for MRouting instance, so no parameters for input waypoint here.
		//@param  abstractMap defines the abstract map that the routing should be applied to.
		//path is an output parameter to return MRoutePath.
		//@return true if success, false if not.
		bool GenerateRoute(const SSD::SimVector<MAbstractRoadInfo>& abstractMap, MRoutePath& path) const;

		//@brief Locate a reflected position on MRoutePath by specifying an input point which can reflect to that MRoutePath.
		//Waypoint list are prebuilt for MRouting instance, so no parameters for input waypoint here.
		//@param inputPt is a 3d point. path is an existing MRoutePath.
		//location is an output parameter that defines the reflected position on 'path'.
		//dir is an output parameter that defines the tangent direction of 'location'.
		//@return true if success, false if not.
		bool LocateInRoutePath(const SSD::SimPoint3D& inputPt, const MRoutePath& path, SSD::SimPoint3D& location, SSD::SimPoint3D& dir) const;

		//@brief Extended version to generate routing organized as a smooth path. The path is a fitted curve based on the result of GenerateRouteTopo().
		//Waypoint list are prebuilt for MRouting instance, so no parameters here.
		//path is an output parameter to return MRoutePathEx.
		//@return true if success, false if not.
		bool GenerateRouteEx(MRoutePathEx& path) const;

		//@brief Extended version to generate routing organized as a smooth path, only when all input sample nodes are valid on driving lane.
		//The path is a fitted curve based on the result of GenerateRouteTopo().
		//@param sampleNodes is waypoint list as input to generate routing.
		//indexOfValidPoints is an output parameter to return the valid points' index regarding to the input vector sampleNodes,
		//so that API user would know about the invalid sample nodes.
		//path is an output parameter to return MRoutePathEx.
		//@return true if success, false if not.
		bool GenerateRouteEx(const SSD::SimPoint3DVector& sampleNodes, SSD::SimVector<int>& indexOfValidPoints, MRoutePathEx& path) const;

	private:
		const MHDMapTS& hdmap;
	};
}