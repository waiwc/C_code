// 
// Created By: Qilin.Ren 2018/05/29
// 

#include "glm/dvec2.hpp"
#include <cmath>

CYBERTRON_BEGIN

CybertronCoreAPI double dot(const dvec2& lhs, const dvec2& rhs)
{
	return lhs.x * rhs.x + lhs.y * rhs.y;
}

CybertronCoreAPI dvec2 normalize(const dvec2& v)
{
	double l = length(v);
	return dvec2(v.x / l, v.y / l);
}

CybertronCoreAPI double distance(const dvec2& v1, const dvec2& v2)
{
	return length(v1 - v2);
}

CybertronCoreAPI double length(const dvec2& v)
{
	return std::sqrt(dot(v, v));
}

CYBERTRON_END
