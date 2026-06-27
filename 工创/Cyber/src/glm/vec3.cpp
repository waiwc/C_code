// 
// Created By: Qilin.Ren 2018/05/29
// 

#include "glm/vec3.hpp"
#include "glm/vec4.hpp"
#include <cmath>
#include <algorithm>

CYBERTRON_BEGIN

CybertronCoreAPI vec3::vec3(const vec4& rhs)
{
	x = rhs.x;
	y = rhs.y;
	z = rhs.z;
}

CybertronCoreAPI vec3 min(const vec3& lhs, const vec3& rhs)
{
	return vec3(
		std::min(lhs.x, rhs.x),
		std::min(lhs.y, rhs.y),
		std::min(lhs.z, rhs.z));
}

CybertronCoreAPI vec3 max(const vec3& lhs, const vec3& rhs)
{
	return vec3(
		std::max(lhs.x, rhs.x),
		std::max(lhs.y, rhs.y),
		std::max(lhs.z, rhs.z));
}

CybertronCoreAPI vec3 operator * (float lhs, const vec3& rhs)
{
	return vec3(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
}

CybertronCoreAPI vec3 operator / (float lhs, const vec3& rhs)
{
	return vec3(lhs / rhs.x, lhs / rhs.y, lhs / rhs.z);
}

CybertronCoreAPI vec3 cross(const vec3& lhs, const vec3& rhs)
{
	return vec3(
		lhs.y * rhs.z - rhs.y * lhs.z,
		lhs.z * rhs.x - rhs.z * lhs.x,
		lhs.x * rhs.y - rhs.x * lhs.y);
}

CybertronCoreAPI float dot(const vec3& lhs, const vec3& rhs)
{
	return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

CybertronCoreAPI vec3 normalize(const vec3& v)
{
	float l = length(v);
	return vec3(v.x / l, v.y / l, v.z / l);
}

CybertronCoreAPI float length(const vec3& v)
{
	return sqrtf(dot(v, v));
}

CybertronCoreAPI float distance(const vec3& v1, const vec3& v2)
{
	return length(v1 - v2);
}

CybertronCoreAPI bvec3 equal(const vec3& lhs, const vec3& rhs)
{
	return bvec3(lhs.x == rhs.x, lhs.y == rhs.y, lhs.z == rhs.z);
}

CYBERTRON_END
