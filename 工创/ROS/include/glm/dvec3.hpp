// 
// Created By: Qilin.Ren 2018/05/29
// 

#pragma once

#include "dvec2.hpp"
#include "bvec3.hpp"
#ifndef CYBERTRON_WIN
#include "stdlib.h"
#endif

CYBERTRON_BEGIN

struct dvec3
{
public:
	double x, y, z;

	dvec3()
	{
	}

	dvec3(const dvec2& xy, double zz)
	{
		x = xy.x;
		y = xy.y;
		z = zz;
	}

	explicit dvec3(double f)
	{
		x = f;
		y = f;
		z = f;
	}

	dvec3(double xx, double yy, double zz)
	{
		x = xx;
		y = yy;
		z = zz;
	}

	dvec3(const dvec3& rhs)
	{
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
	}

	dvec3(const double* p)
	{
		x = p[0];
		y = p[1];
		z = p[2];
	}

	dvec3(const float* p)
	{
		x = (double)p[0];
		y = (double)p[1];
		z = (double)p[2];
	}

	CybertronCoreAPI dvec3(const dvec4& rhs);

	~dvec3()
	{
	}

	double& operator[](size_t index)
	{
		return (&x)[index];
	}

	double operator[](size_t index) const
	{
		return (&x)[index];
	}

	bool operator == (const dvec3& rhs) const
	{
		return x == rhs.x && y == rhs.y && z == rhs.z;
	}

	bool operator != (const dvec3& rhs) const
	{
		return !(*this == rhs);
	}

	dvec3 operator + (const dvec3& rhs) const
	{
		return dvec3(x + rhs.x, y + rhs.y, z + rhs.z);
	}

	dvec3 operator - (const dvec3& rhs) const
	{
		return dvec3(x - rhs.x, y - rhs.y, z - rhs.z);
	}

	dvec3 operator + (double rhs) const
	{
		return dvec3(x + rhs, y + rhs, z + rhs);
	}

	dvec3 operator - (double rhs) const
	{
		return dvec3(x - rhs, y - rhs, z - rhs);
	}

	dvec3 operator * (double f) const
	{
		return dvec3(x * f, y * f, z * f);
	}

	dvec3 operator / (double f) const
	{
		return dvec3(x / f, y / f, z / f);
	}

	dvec3 operator * (const dvec3& rhs) const
	{
		return dvec3(x * rhs.x, y * rhs.y, z * rhs.z);
	}

	dvec3 operator / (const dvec3& rhs) const
	{
		return dvec3(x / rhs.x, y / rhs.y, z / rhs.z);
	}

	dvec3 operator -() const
	{
		return dvec3(-x, -y, -z);
	}

	dvec3& operator += (const dvec3& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;

		return *this;
	}

	dvec3& operator -= (const dvec3& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;

		return *this;
	}

	dvec3& operator *= (const dvec3& rhs)
	{
		x *= rhs.x;
		y *= rhs.y;
		z *= rhs.z;

		return *this;
	}

	dvec3& operator /= (const dvec3& rhs)
	{
		x /= rhs.x;
		y /= rhs.y;
		z /= rhs.z;

		return *this;
	}

	dvec3& operator *= (double rhs)
	{
		x *= rhs;
		y *= rhs;
		z *= rhs;

		return *this;
	}

	dvec3& operator /= (double rhs)
	{
		x /= rhs;
		y /= rhs;
		z /= rhs;

		return *this;
	}
};

inline double* value_ptr(dvec3& m)
{
	return (double*)(&m.x);
}

inline const double* value_ptr(const dvec3& m)
{
	return (const double*)(&m.x);
}

inline dvec3 make_vec3(const double* pValues)
{
	return dvec3(pValues[0], pValues[1], pValues[2]);
}

inline dvec3 mix(const dvec3& val1, const dvec3& val2, float a)
{
	return val1 * (1 - a) + val2 * a;
}

CybertronCoreAPI dvec3 operator * (double lhs, const dvec3& rhs);

CybertronCoreAPI dvec3 operator / (double lhs, const dvec3& rhs);

CybertronCoreAPI dvec3 cross(const dvec3& lhs, const dvec3& rhs);

CybertronCoreAPI double dot(const dvec3& lhs, const dvec3& rhs);

CybertronCoreAPI dvec3 normalize(const dvec3& v);

CybertronCoreAPI double length(const dvec3& v);

CybertronCoreAPI double distance(const dvec3& v1, const dvec3& v2);

CybertronCoreAPI bvec3 equal(const dvec3& lhs, const dvec3& rhs);

CYBERTRON_END
