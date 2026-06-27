// 
// Created By: Qilin.Ren 2018/05/29
// 

#pragma once

#include "../DefinesCore.hpp"
#ifndef CYBERTRON_WIN
#include "stdlib.h"
#endif

CYBERTRON_BEGIN

struct dquat
{
public:
	double x, y, z, w;

	dquat()
	{
	}

	CybertronCoreAPI explicit dquat(const dmat3& m);

	CybertronCoreAPI explicit dquat(const dmat4& m);

	CybertronCoreAPI dquat(const dvec3& from, const dvec3& to);

	CybertronCoreAPI dquat(const dvec3& eulerAngles);

	dquat(double ww, double xx, double yy, double zz)
	{
		x = xx;
		y = yy;
		z = zz;
		w = ww;
	}

	dquat(const dquat& rhs)
	{
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		w = rhs.w;
	}

	~dquat()
	{
	}

	bool operator == (const dquat& rhs) const
	{
		return x == rhs.x && y == rhs.y && z == rhs.z && w == rhs.w;
	}

	bool operator != (const dquat& rhs) const
	{
		return !(*this == rhs);
	}

	double& operator[](size_t index)
	{
		return (&x)[index];
	}

	double operator[](size_t index) const
	{
		return (&x)[index];
	}

	CybertronCoreAPI dquat operator * (const dquat& rhs) const;

	CybertronCoreAPI dquat operator * (double rhs) const;

	CybertronCoreAPI dquat operator / (double rhs) const;

	CybertronCoreAPI dquat operator + (const dquat& rhs) const;

	CybertronCoreAPI dquat operator - (const dquat& rhs) const;

	CybertronCoreAPI dquat operator - () const;

	CybertronCoreAPI dquat& operator *= (const dquat& rhs);

	CybertronCoreAPI dvec3 operator * (const dvec3& rhs) const;
};

inline double* value_ptr(dquat& m)
{
	return (double*)(&m.x);
}

inline const double* value_ptr(const dquat& m)
{
	return (const double*)(&m.x);
}

CybertronCoreAPI dquat operator * (double lhs, const dquat& rhs);

CybertronCoreAPI double dot(const dquat& q1, const dquat& q2);

CybertronCoreAPI double length(const dquat& q);

CybertronCoreAPI dquat quat_cast(const dmat3& m);

CybertronCoreAPI dquat quat_cast(const dmat4& m);

CybertronCoreAPI dmat3 mat3_cast(const dquat& q);

CybertronCoreAPI dmat4 mat4_cast(const dquat& q);

CybertronCoreAPI dquat lerp(const dquat& q1, const dquat& q2, double a);

CybertronCoreAPI dquat slerp(const dquat& q1, const dquat& q2, double a);

CybertronCoreAPI dvec3 eulerAngles(const dquat& q);

CybertronCoreAPI dmat4 eulerAngleXYZ(double x, double y, double z);

CybertronCoreAPI dquat normalize(const dquat& q);

CybertronCoreAPI dquat conjugate(const dquat& q);

CybertronCoreAPI dquat inverse(const dquat& q);

CybertronCoreAPI dquat cross(const dquat& q1, const dquat& q2);

CybertronCoreAPI dquat angleAxis(double angle, const dvec3& dir);

CYBERTRON_END
