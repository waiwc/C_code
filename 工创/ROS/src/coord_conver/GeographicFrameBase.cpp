
#include "coord_conver/GeographicFrameBase.h"
#include "coord_conver/GIS.h"
#include "coord_conver/dvec3.h"
#include "coord_conver/GeographicFrameEnu.h"

std::shared_ptr<GeographicFrameBase> GeographicFrameBase::CreateEnuFromWgs(
    double centerlat,
    double centerlon,
	double centeralt)
{
    dvec3 ecef = GIS::WGSToEarthCenterXYZ(centerlat, centerlon, centeralt);
    return std::shared_ptr<GeographicFrameEnu>(new GeographicFrameEnu(
        GIS::BuildEnuCoordinateSystem(ecef)));
}
