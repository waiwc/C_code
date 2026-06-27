#pragma once

#include "libexport.h"

#ifdef STATIC_SSD
#include "SSD/public/SimString.h"
#else
#include "SSD/SimString.h"
#endif
#include "public/MProtoDataStruct.h"

namespace rt {
    namespace hdmap
    {
        class Lane;
        class Crosswalk;
        class Stopline;
        class ParkingSpace;
        enum Lane_LaneType;
    }
}

namespace HDMapStandalone {
    class LIBEXPORT MHDMapProto {
    public:
        //@brief Get lane's sample point info.
        //@param laneName with this format roadId_sectionIndex_laneId.
        //@pre-condition laneName must exist in current map. Please use API MHDMap::ContainsLane().
        //@return Returns ProtoLaneInfo object.
        static ProtoLaneInfo GetLaneSample(const SSD::SimString& laneName);

        //@brief Get traffic lights that are in logic bound with the specified lane.
        //@param laneName with this format roadId_sectionIndex_laneId.
        //@pre-condition laneName must exist in current map. Please use API MHDMap::ContainsLane().
        //@return Returns MSignalVector object.
        static SSD::SimVector<rt::hdmap::Signal> GetTrafficLights(const SSD::SimString& laneName);

        //@brief Get traffic light by id.
        //@param id is traffic light's id.
        //@pre-condition id must exist in current map.
        //@return Returns the reference to a rt::hdmap::Signal object.
        static const rt::hdmap::Signal& GetTrafficLight(const SSD::SimString& id);

        //@brief Get lane according to laneName.
        //@param laneName with this format roadId_sectionIndex_laneId.
        //@pre-condition laneName must exist in current map. Please use API MHDMap::ContainsLane().
        //@return Returns Lane object.
        static const rt::hdmap::Lane& GetLane(const SSD::SimString& laneName);

        //@brief Get lane's type according to laneName.
        //@param laneName with this format roadId_sectionIndex_laneId.
        //@pre-condition laneName must exist in current map. Please use API MHDMap::ContainsLane().
        //@return Returns Lane_LaneType object.
        static const rt::hdmap::Lane_LaneType& GetLaneType(const SSD::SimString& laneName);

        //@brief Get crosswalk according to object id.
        //@param crosswalk's object id.
        //@pre-condition id must exist in current map.
        //@return Returns Crosswalk object.
        static const rt::hdmap::Crosswalk& GetCrosswalk(const SSD::SimString& id);

        //@brief Get stopLine according to object id.
        //@param stopLine's object id.
        //@pre-condition id must exist in current map.
        //@return Returns Stopline object.
        static const rt::hdmap::Stopline& GetStopline(const SSD::SimString& id);

        //@brief Get parkingSpace according to object id.
        //@param parkingSpace's object id.
        //@pre-condition id must exist in current map.
        //@return Returns ParkingSpace object.
        static const rt::hdmap::ParkingSpace& GetParkingSpace(const SSD::SimString& id);
    };
}