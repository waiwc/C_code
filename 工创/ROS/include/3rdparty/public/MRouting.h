
#pragma once

#include "libexport.h"
#include "common/MRouteElement.h"
#include "common/MRoutePath.h"
#include "common/MAbstractRoadInfo.h"

namespace HDMapStandalone {

class LIBEXPORT MRouting {
public:
    //@brief Validate if the input point is valid on or close to a driving lane.
    //@param [x, y] represents a 2d point.
    //@return Returns true if valid, false if not.
    static bool ValidatePoint(const double& x, const double& y);

	//@brief Validate if the input point is valid on or close to a driving lane.
	//@param pt represents a 3d point.
	//laneName as an output parameter that returns laneName that input pt is inside.
	//s as an output parameter that returns s value in [s, t] coordinate system.
	//@return Returns true if valid, false if not.
	static bool ValidatePoint(const SSD::SimPoint3D& pt, SSD::SimString& laneName, double& s);

    //@brief Get waypoint size of MRouting instance.
    //@return Returns waypoint size.
    static int GetPointSize();

    //@brief Add waypoint into MRouting instance.
    //@param [x, y] represents a 2d point.
    //@return Returns true if succeeded, false if failed.
    static bool AddPoint(const double& x, const double& y);

	//@brief Add waypoint into MRouting instance.
	//@param pt represents a 3d point.
	//@return Returns true if succeeded, false if failed.
	static bool AddPoint(const SSD::SimPoint3D& pt);

	//@brief Add waypoint into MRouting instance.
	//@param pt represents a 3d point. laneName is specified lane's id to add point onto, formatted as roadId_sectionIndex_laneId.
	//s is the value in [s, t] coordinate system.
	static void AddPoint(const SSD::SimPoint3D& pt, const SSD::SimString& laneName, const double& s);

    //@brief Remove last waypoint of MRouting instance.
    //@return Returns true if succeeded, false when the waypoint list is already empty.
    static bool RemoveLastPoint();

    //@brief Clear all waypoints of MRouting instance.
    static void ClearPoints();

    //@brief Generate routing organized from topology nodes.
    //@param Output ASCII file that accepts generated routing topo data(proto data as original)
    //@return Returns list of MRoute object. Returns nullptr if result is empty.
    static bool GenerateRouteTopo(SSD::SimVector<MRouteElement>& routeTopo, const char* outputName = nullptr);

    //@brief Generate routing organized as a smooth path. The path is a fitted curve based on the result of GenerateRouteTopo().
    //Waypoint list are prebuilt for MRouting instance, so no parameters for input waypoint here.
    //path is an output parameter to return MRoutePath.
    //@return true if success, false if not.
    static bool GenerateRoute(MRoutePath& path);

	//@brief Generate routing organized as a smooth path. The path is a fitted curve based on the result of GenerateRouteTopo().
	//Waypoint list are prebuilt for MRouting instance, so no parameters for input waypoint here.
	//path is an output parameter to return MRoutePath.
	//routes is an output parameter to return list of MRouteElement that defines each route.
	//@return true if success, false if not.
	static bool GenerateRoute(MRoutePath& path, SSD::SimVector<MRouteElement>& routes);

    //@brief Generate routing organized as a smooth path, only when all input sample nodes are valid on driving lane.
    //The path is a fitted curve based on the result of GenerateRouteTopo().
    //@param sampleNodes is waypoint list as input to generate routing.
    //indexOfValidPoints is an output parameter to return the valid points' index regarding to the input vector sampleNodes,
    //so that API user would know about the invalid sample nodes.
    //path is an output parameter to return MRoutePath.
    //@return true if success, false if not.
    static bool GenerateRoute(const SSD::SimPoint3DVector& sampleNodes, SSD::SimVector<int>& indexOfValidPoints, MRoutePath& path);

	//@brief Generate routing organized as a smooth path.
	//It respects sampleNodes' height value when considering sample node's validity.
	//The path is a fitted curve based on the result of GenerateRouteTopo().
	//@param sampleNodes is waypoint list as input to generate routing.
	//indexOfValidPoints is an output parameter to return the valid points' index regarding to the input vector sampleNodes,
	//so that API user would know about the invalid sample nodes.
	//path is an output parameter to return MRoutePath.
	//@return true if success, false if not.
	static bool GenerateRouteWithHeight(const SSD::SimPoint3DVector& sampleNodes, SSD::SimVector<int>& indexOfValidPoints, MRoutePath& path);

	//@brief Generate routing organized as a smooth path.
	//The path is a fitted curve based on the result of GenerateRouteTopo().
	//@param sampleNodes is waypoint list as input to generate routing.
	//indexOfValidPoints is an output parameter to return the valid points' index regarding to the input vector sampleNodes,
	//so that API user would know about the invalid sample nodes.
	//path is an output parameter to return MRoutePath.
	//routePtList is an output parameter to return MRoutePoint list.
	//@return true if success, false if not.
	static bool GenerateRoute(const SSD::SimPoint3DVector& sampleNodes, SSD::SimVector<int>& indexOfValidPoints,
		MRoutePath& path, SSD::SimVector<MRoutePoint>& routePtList);

	//@brief Generate best routing organized as a smooth path. It will pick one as best if there are multiple possibilities for input sample nodes.
	//It respects sampleNodes' height value when considering sample node's validity.
	//The path is a fitted curve based on the result of GenerateRouteTopo().
	//@param sampleNodes is waypoint list as input to generate routing.
	//indexOfValidPoints is an output parameter to return the valid points' index regarding to the input vector sampleNodes,
	//so that API user would know about the invalid sample nodes.
	//path is an output parameter to return MRoutePath.
	//routePtList is an output parameter to return MRoutePoint list.
	//@return true if success, false if not.
	static bool GenerateBestRoute(const SSD::SimPoint3DVector& sampleNodes, SSD::SimVector<int>& indexOfValidPoints,
		MRoutePath& path, SSD::SimVector<MRoutePoint>& routePtList);

    //@brief Generate routing organized as a smooth path based on input MAbstractRoadInfoVector.
    //Waypoint list are prebuilt for MRouting instance, so no parameters for input waypoint here.
    //@param  abstractMap defines the abstract map that the routing should be applied to.
    //path is an output parameter to return MRoutePath.
    //@return true if success, false if not.
    static bool GenerateRoute(const SSD::SimVector<MAbstractRoadInfo>& abstractMap, MRoutePath& path);

    //@brief Locate a reflected position on MRoutePath by specifying an input point which can reflect to that MRoutePath.
    //Waypoint list are prebuilt for MRouting instance, so no parameters for input waypoint here.
    //@param inputPt is a 3d point. path is an existing MRoutePath.
    //location is an output parameter that defines the reflected position on 'path'.
    //dir is an output parameter that defines the tangent direction of 'location'.
    //@return true if success, false if not.
    static bool LocateInRoutePath(const SSD::SimPoint3D& inputPt, const MRoutePath& path, SSD::SimPoint3D& location, SSD::SimPoint3D& dir);

    //@brief Extended version to generate routing organized as a smooth path. The path is a fitted curve based on the result of GenerateRouteTopo().
    //Waypoint list are prebuilt for MRouting instance, so no parameters here.
    //path is an output parameter to return MRoutePathEx.
    //@return true if success, false if not.
    static bool GenerateRouteEx(MRoutePathEx& path);

    //@brief Extended version to generate routing organized as a smooth path, only when all input sample nodes are valid on driving lane.
    //The path is a fitted curve based on the result of GenerateRouteTopo().
    //@param sampleNodes is waypoint list as input to generate routing.
    //indexOfValidPoints is an output parameter to return the valid points' index regarding to the input vector sampleNodes,
    //so that API user would know about the invalid sample nodes.
    //path is an output parameter to return MRoutePathEx.
    //@return true if success, false if not.
    static bool GenerateRouteEx(const SSD::SimPoint3DVector& sampleNodes, SSD::SimVector<int>& indexOfValidPoints, MRoutePathEx& path);
};
}
