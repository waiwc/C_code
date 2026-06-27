// 
// Created By: Qilin.Ren 2018/05/29
// 

#include "glm/vec3.hpp"
#include "glm/vec4.hpp"
#include <cmath>

CYBERTRON_BEGIN

CybertronCoreAPI vec4::vec4(const vec3& rhs, float ww)
{
	x = rhs.x;
	y = rhs.y;
	z = rhs.z;
	w = ww;
}

CybertronCoreAPI vec3 vec4::xyz() const
{
	return vec3(x, y, z);
}

CybertronCoreAPI vec4 operator * (float lhs, const vec4& rhs)
{
	return vec4(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z, lhs * rhs.w);
}

CybertronCoreAPI vec4 normalize(const vec4& v)
{
	float lengthSq = v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w;
	float len = sqrtf(lengthSq);
	float invLen = 1 / len;
	return vec4(v.x * invLen, v.y * invLen, v.z * invLen, v.w * invLen);
}

CYBERTRON_END
