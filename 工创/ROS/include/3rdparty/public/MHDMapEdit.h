#pragma once

#include "libexport.h"
#include "public/common/MLaneInfo.h"

namespace HDMapStandalone {
    struct LIBEXPORT RoadTemplate
    {
        //@brief lane count at right side.
        int laneCountRight = 1;

        //@brief lane count at left side.
        int laneCountLeft = 1;

        //@brief lane width in meter.
        double laneWidth = 3.5;

        //@brief flag of shoulder existence.
        bool hasShoulder = false;

        //@brief shoulder lane's width in meter.
        double shoulderWidth = 2;
    };

    class LIBEXPORT MHDMapEdit
    {
    public:
        //@brief Add a road based on reference line's knots and tangents, and also based on RoadTemplate coniguration.
        //Knots and tangents are based on localENU.
        //@param centerLineknots represents reference line's knots.
        //centerLineTangents represents reference line's tangents at each knot.
        //roadTemplate is the RoadTemplate coniguration.
        //newRoad is an output parameter that returns the id of new added road.
        //@return Returns true if succeeded, false if failed.
        static bool AddRoad_V0(const SSD::SimPoint3DVector& centerLineknots, const SSD::SimPoint3DVector& centerLineTangents,
            const RoadTemplate& roadTemplate, long& newRoad);

        //@brief Add a road based on reference line's knots and tangents, and also based on RoadTemplate coniguration.
        //Knots and tangents are based on localENU.
        //@param centerLineknots represents reference line's knots.
        //centerLineTangents represents reference line's tangents at each knot.
        //roadTemplate is the RoadTemplate coniguration.
        //newRoad is an output parameter that returns the id of new added road.
        //laneNameList is an output parameter that returns names of all lanes that belong to the new added road.
        //@return Returns true if succeeded, false if failed.
        static bool AddRoad(const SSD::SimPoint3DVector& centerLineknots, const SSD::SimPoint3DVector& centerLineTangents,
            const RoadTemplate& roadTemplate, long& newRoad, SSD::SimStringVector& laneNameList);

        //@brief Auto detect junction's incoming roads by algorithm regarding to junction's location and its neighborhood.
        //@param junctionCenter represents junction center point's location.
        //autoDetectedRoads as an output parameter, returns detected incoming road id list.
        //junctionId as an output parametr, returns juction's unique id in map.
        //@return Returns true if succeeded, false if failed.
        static bool AutoDetectIncomingRoad(const SSD::SimPoint3D& junctionCenter, SSD::SimVector<long>& autoDetectedRoads, long& junctionId);

        //@brief Auto build junction's roads by algorithm regarding to junction's incoming roads, defined by API AutoDetectIncomingRoad().
        //@param junctionId represents junction's id in map.
        //junctionRoadIdList as an output parameter, returns newly build junction roads' id list.
        //laneNameList as an output parametr, returns juction roads's lane name list.
        //@return Returns true if succeeded, false if failed.
        static bool AutoBuildJunctionRoad(const long& junctionId, SSD::SimVector<long>& junctionRoadIdList,
            SSD::SimStringVector& laneNameList);
    };
}