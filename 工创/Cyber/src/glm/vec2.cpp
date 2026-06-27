// 
// Created By: Qilin.Ren 2018/05/29
// 

#include "glm/vec2.hpp"
#include <cmath>

CYBERTRON_BEGIN

CybertronCoreAPI float dot(const vec2& lhs, const vec2& rhs)
{
	return lhs.x * rhs.x + lhs.y * rhs.y;
}

CybertronCoreAPI vec2 normalize(const vec2& v)
{
	float l = length(v);
	return vec2(v.x / l, v.y / l);
}

CybertronCoreAPI double distance(const vec2& v1, const vec2& v2)
{
	return length(v1 - v2);
}

CybertronCoreAPI float length(const vec2& v)
{
	return sqrtf(dot(v, v));
}

CYBERTRON_END
