// 
// Created By: Qilin.Ren 2018/06/07
// 

#pragma once

#include "vec3.hpp"

CYBERTRON_BEGIN

struct box3
{
public:
	vec3 minVec;
	vec3 maxVec;

	CybertronCoreAPI box3();
	CybertronCoreAPI box3(const box3& rhs);
	CybertronCoreAPI box3(const vec3& _minVec, const vec3& _maxVec);
	CybertronCoreAPI box3& operator = (const box3& rhs);
	CybertronCoreAPI ~box3();

	CybertronCoreAPI bool contains(const vec3& v) const;
	CybertronCoreAPI void clear();
	CybertronCoreAPI bool empty() const;

	CybertronCoreAPI void enlarge(const box3& rhs);
	CybertronCoreAPI void enlarge(const vec3& v);

	CybertronCoreAPI void transform(const mat4& matTransform);

	inline vec3 center() const
	{
		return minVec + (maxVec - minVec) / 2;
	}

	inline vec3 size() const
	{
		return maxVec - minVec;
	}

	inline box3& operator |= (const box3& rhs) {
		enlarge(rhs);
		return *this;
	}

	inline box3& operator |= (const vec3& rhs) {
		enlarge(rhs);
		return *this;
	}
};

inline box3 operator | (const box3& lhs, const box3& rhs)
{
	box3 result = lhs;
	result.enlarge(rhs);
	return result;
}

inline box3 operator | (const box3& lhs, const vec3& rhs)
{
	box3 result = lhs;
	result.enlarge(rhs);
	return result;
}

CYBERTRON_END
