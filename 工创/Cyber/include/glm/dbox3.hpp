// 
// Created By: Qilin.Ren 2018/06/07
// 

#pragma once

#include "dvec3.hpp"

CYBERTRON_BEGIN

struct dbox3
{
public:
	dvec3 minVec;
	dvec3 maxVec;

	CybertronCoreAPI dbox3();
	CybertronCoreAPI dbox3(const dbox3& rhs);
	CybertronCoreAPI dbox3(const dvec3& _minVec, const dvec3& _maxVec);
	CybertronCoreAPI dbox3& operator = (const dbox3& rhs);
	CybertronCoreAPI ~dbox3();

	CybertronCoreAPI bool contains(const dvec3& v) const;
	CybertronCoreAPI void clear();
	CybertronCoreAPI bool empty() const;

	CybertronCoreAPI void enlarge(const dbox3& rhs);
	CybertronCoreAPI void enlarge(const dvec3& v);

	CybertronCoreAPI void transform(const dmat4& matTransform);

	inline dvec3 center() const
	{
		return minVec + (maxVec - minVec) / 2;
	}

	inline dvec3 size() const
	{
		return maxVec - minVec;
	}

	inline dbox3& operator |= (const dbox3& rhs) {
		enlarge(rhs);
		return *this;
	}

	inline dbox3& operator |= (const dvec3& rhs) {
		enlarge(rhs);
		return *this;
	}
};

inline dbox3 operator | (const dbox3& lhs, const dbox3& rhs)
{
	dbox3 result = lhs;
	result.enlarge(rhs);
	return result;
}

inline dbox3 operator | (const dbox3& lhs, const dvec3& rhs)
{
	dbox3 result = lhs;
	result.enlarge(rhs);
	return result;
}

CYBERTRON_END
