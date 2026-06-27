// 
// Created By: Qilin.Ren 2018/05/29
// 

#include "glm/dvec3.hpp"
#include "glm/dvec4.hpp"
#include <cmath>

CYBERTRON_BEGIN

CybertronCoreAPI dvec3::dvec3(const dvec4& rhs)
{
	x = rhs.x;
	y = rhs.y;
	z = rhs.z;
}

CybertronCoreAPI dvec3 operator * (double lhs, const dvec3& rhs)
{
	return dvec3(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
}

CybertronCoreAPI dvec3 operator / (double lhs, const dvec3& rhs)
{
	return dvec3(lhs / rhs.x, lhs / rhs.y, lhs / rhs.z);
}

CybertronCoreAPI dvec3 cross(const dvec3& lhs, const dvec3& rhs)
{
	return dvec3(
		lhs.y * rhs.z - rhs.y * lhs.z,
		lhs.z * rhs.x - rhs.z * lhs.x,
		lhs.x * rhs.y - rhs.x * lhs.y);
}

CybertronCoreAPI double dot(const dvec3& lhs, const dvec3& rhs)
{
	return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

CybertronCoreAPI dvec3 normalize(const dvec3& v)
{
	double l = length(v);
	return dvec3(v.x / l, v.y / l, v.z / l);
}

CybertronCoreAPI double length(const dvec3& v)
{
	return std::sqrt(dot(v, v));
}

CybertronCoreAPI double distance(const dvec3& v1, const dvec3& v2)
{
	return length(v1 - v2);
}

CybertronCoreAPI bvec3 equal(const dvec3& lhs, const dvec3& rhs)
{
	return bvec3(lhs.x == rhs.x, lhs.y == rhs.y, lhs.z == rhs.z);
}

CYBERTRON_END
