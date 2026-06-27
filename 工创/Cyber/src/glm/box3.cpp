// 
// Created By: Qilin.Ren 2018/06/07
// 

#include "glm/box3.hpp"
#include "glm/mat4.hpp"
#include <limits>

CYBERTRON_BEGIN

CybertronCoreAPI box3::box3()
{
	clear();
}

CybertronCoreAPI box3::box3(const box3& rhs)
{
	minVec = rhs.minVec;
	maxVec = rhs.maxVec;
}

CybertronCoreAPI box3::box3(const vec3& _minVec, const vec3& _maxVec)
{
	minVec = _minVec;
	maxVec = _maxVec;
}

CybertronCoreAPI box3& box3::operator = (const box3& rhs)
{
	minVec = rhs.minVec;
	maxVec = rhs.maxVec;

	return *(this);
}

CybertronCoreAPI box3::~box3()
{
	// do nothing.
}

CybertronCoreAPI bool box3::contains(const vec3& v) const
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

CybertronCoreAPI void box3::clear()
{
	minVec.x = minVec.y = minVec.z = std::numeric_limits<float>::max();
	maxVec.x = maxVec.y = maxVec.z = -std::numeric_limits<float>::max();
}

CybertronCoreAPI bool box3::empty() const
{
	return minVec.x >= maxVec.x;
}

CybertronCoreAPI void box3::enlarge(const box3& rhs)
{
	enlarge(rhs.minVec);
	enlarge(rhs.maxVec);
}

CybertronCoreAPI void box3::enlarge(const vec3& v)
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

CybertronCoreAPI void box3::transform(const mat4& matTransform)
{
	box3 result;
	vec3 s = size();

	vec3 v = matTransform * minVec;
	result.enlarge(v);
	v = matTransform * (minVec + vec3(s.x, 0, 0));
	result.enlarge(v);
	v = matTransform * (minVec + vec3(0, s.y, 0));
	result.enlarge(v);
	v = matTransform * (minVec + vec3(0, 0, s.z));
	result.enlarge(v);
	v = matTransform * (minVec + vec3(s.x, s.y, 0));
	result.enlarge(v);
	v = matTransform * (minVec + vec3(0, s.y, s.z));
	result.enlarge(v);
	v = matTransform * (minVec + vec3(s.x, 0, s.z));
	result.enlarge(v);
	v = matTransform * (maxVec);
	result.enlarge(v);

	minVec = result.minVec;
	maxVec = result.maxVec;
}

CYBERTRON_END
