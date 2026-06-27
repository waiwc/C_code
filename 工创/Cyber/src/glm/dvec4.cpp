// 
// Created By: Qilin.Ren 2018/05/29
// 

#include "glm/dvec3.hpp"
#include "glm/dvec4.hpp"
#include <cmath>

CYBERTRON_BEGIN

CybertronCoreAPI dvec4::dvec4(const dvec3& rhs, double ww)
{
	x = rhs.x;
	y = rhs.y;
	z = rhs.z;
	w = ww;
}

CybertronCoreAPI dvec3 dvec4::xyz() const
{
	return dvec3(x, y, z);
}

CybertronCoreAPI dvec4 operator * (double lhs, const dvec4& rhs)
{
	return dvec4(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z, lhs * rhs.w);
}

CybertronCoreAPI dvec4 normalize(const dvec4& v)
{
	double lengthSq = v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w;
	double len = std::sqrt(lengthSq);
	double invLen = 1 / len;
	return dvec4(v.x * invLen, v.y * invLen, v.z * invLen, v.w * invLen);
}

CYBERTRON_END
