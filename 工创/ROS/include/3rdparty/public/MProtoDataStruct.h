#pragma once

#ifdef STATIC_SSD
#include "SSD/public/SimVector.h"
#else
#include "SSD/SimVector.h"
#endif
#include "map/proto/map_signal.pb.h"
#include "map/proto/map_lane.pb.h"

namespace HDMapStandalone {

    struct LIBEXPORT ProtoLaneInfo
    {
        //@brief Curve data of lane's left boundary.
        rt::hdmap::Curve leftBoundary;

        //@brief Curve data of lane's right boundary.
        rt::hdmap::Curve rightBoundary;

        //@brief Curve data of lane's center line.
        rt::hdmap::Curve centerLine;
    };
}
