
#include "coord_conver/tsf.h"
#include "coord_conver/GIS.h"
#include "coord_conver/GeographicFrameBase.h"

static std::shared_ptr<GeographicFrameBase> mGeoFrame = nullptr;

void SetGPSBase(double lat, double lon, double alt)
{
	mGeoFrame = GeographicFrameBase::CreateEnuFromWgs(lat, lon, alt);
}

void GetLocal(double lat, double lon, double alt, Point& local)
{
	dvec3 wgsPosition(lat, lon, alt);
	dvec3 pos = mGeoFrame->WgsToLocal(wgsPosition);
	local.x = pos.x;
	local.y = pos.y;
	local.z = pos.z;
}

Point GetWGS(const Point& pos) {
	dvec3 p(pos.x, pos.y, pos.z);
	const dvec3& wgs = mGeoFrame->LocalToWgs(p);
	Point ret;
	ret.x = wgs.x;
	ret.y = wgs.y;
	ret.z = wgs.z;
	return std::move(ret);
}
