// 
// Created By: Qilin.Ren 2018/06/07
// 

#include "glm/dbox3.hpp"
#include "glm/dmat4.hpp"
#include <limits>

CYBERTRON_BEGIN

CybertronCoreAPI dbox3::dbox3()
{
	clear();
}

CybertronCoreAPI dbox3::dbox3(const dbox3& rhs)
{
	minVec = rhs.minVec;
	maxVec = rhs.maxVec;
}

CybertronCoreAPI dbox3::dbox3(const dvec3& _minVec, const dvec3& _maxVec)
{
	minVec = _minVec;
	maxVec = _maxVec;
}

CybertronCoreAPI dbox3& dbox3::operator = (const dbox3& rhs)
{
	minVec = rhs.minVec;
	maxVec = rhs.maxVec;

	return *(this);
}

CybertronCoreAPI dbox3::~dbox3()
{
	// do nothing.
}

CybertronCoreAPI bool dbox3::contains(const dvec3& v) const
{
	if (v.x < minVec.x || v.x > maxVec.x)
	{
		return false;
	}
	if (v.y < minVec.y || v.y > maxVec.y)
	{
		return false;
	}
	if (v.z < minVec.z || v.z > maxVec.z)
	{
		return false;
	}

	return true;
}

CybertronCoreAPI void dbox3::clear()
{
	minVec.x = minVec.y = minVec.z = std::numeric_limits<double>::max();
	maxVec.x = maxVec.y = maxVec.z = -std::numeric_limits<double>::max();
}

CybertronCoreAPI bool dbox3::empty() const
{
	return minVec.x >= maxVec.x;
}

CybertronCoreAPI void dbox3::enlarge(const dbox3& rhs)
{
	enlarge(rhs.minVec);
	enlarge(rhs.maxVec);
}

CybertronCoreAPI void dbox3::enlarge(const dvec3& v)
{
	if (v.x < minVec.x)
	{
		minVec.x = v.x;
	}
	if (v.y < minVec.y)
	{
		minVec.y = v.y;
	}
	if (v.z < minVec.z)
	{
		minVec.z = v.z;
	}

	if (v.x > maxVec.x)
	{
		maxVec.x = v.x;
	}
	if (v.y > maxVec.y)
	{
		maxVec.y = v.y;
	}
	if (v.z > maxVec.z)
	{
		maxVec.z = v.z;
	}
}

CybertronCoreAPI void dbox3::transform(const dmat4& matTransform)
{
	dbox3 result;
	dvec3 s = size();

	dvec3 v = matTransform * minVec;
	result.enlarge(v);
	v = matTransform * (minVec + dvec3(s.x, 0, 0));
	result.enlarge(v);
	v = matTransform * (minVec + dvec3(0, s.y, 0));
	result.enlarge(v);
	v = matTransform * (minVec + dvec3(0, 0, s.z));
	result.enlarge(v);
	v = matTransform * (minVec + dvec3(s.x, s.y, 0));
	result.enlarge(v);
	v = matTransform * (minVec + dvec3(0, s.y, s.z));
	result.enlarge(v);
	v = matTransform * (minVec + dvec3(s.x, 0, s.z));
	result.enlarge(v);
	v = matTransform * (maxVec);
	result.enlarge(v);

	minVec = result.minVec;
	maxVec = result.maxVec;
}

CYBERTRON_END
