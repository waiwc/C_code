// 
// Created By: Qilin.Ren 2018/05/29
// 

#pragma once

#include "../DefinesCore.hpp"
#ifndef CYBERTRON_WIN
#include "stdlib.h"
#endif

CYBERTRON_BEGIN

struct dvec2
{
public:
	double x, y;

	dvec2()
	{
	}

	explicit dvec2(double f)
	{
		x = f;
		y = f;
	}

	dvec2(double xx, double yy)
	{
		x = xx;
		y = yy;
	}

	dvec2(const dvec2& rhs)
	{
		x = rhs.x;
		y = rhs.y;
	}

	~dvec2()
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

	bool operator == (const dvec2& rhs) const
	{
		return x == rhs.x && y == rhs.y;
	}

	bool operator != (const dvec2& rhs) const
	{
		return !(*this == rhs);
	}

	dvec2 operator - (const dvec2& rhs) const
	{
		return dvec2(x - rhs.x, y - rhs.y);
	}

	dvec2 operator + (const dvec2& rhs) const
	{
		return dvec2(x + rhs.x, y + rhs.y);
	}

	dvec2 operator * (double f) const
	{
		return dvec2(x * f, y * f);
	}

	dvec2 operator / (double f) const
	{
		return dvec2(x / f, y / f);
	}

	dvec2& operator += (const dvec2& rhs)
	{
		x += rhs.x;
		y += rhs.y;

		return *this;
	}

	dvec2& operator -= (const dvec2& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;

		return *this;
	}

	dvec2& operator *= (const dvec2& rhs)
	{
		x *= rhs.x;
		y *= rhs.y;

		return *this;
	}

	dvec2& operator /= (const dvec2& rhs)
	{
		x /= rhs.x;
		y /= rhs.y;

		return *this;
	}

	dvec2& operator *= (double rhs)
	{
		x *= rhs;
		y *= rhs;

		return *this;
	}

	dvec2& operator /= (double rhs)
	{
		x /= rhs;
		y /= rhs;

		return *this;
	}
};

inline dvec2 operator * (double lhs, const dvec2& rhs)
{
	return dvec2(lhs * rhs.x, lhs * rhs.y);
}

inline double* value_ptr(dvec2& m)
{
	return (double*)(&m.x);
}

inline const double* value_ptr(const dvec2& m)
{
	return (const double*)(&m.x);
}

inline dvec2 make_vec2(const double* pValues)
{
	return dvec2(pValues[0], pValues[1]);
}

inline dvec2 mix(const dvec2& val1, const dvec2& val2, float a)
{
	return val1 * (1 - a) + val2 * a;
}

CybertronCoreAPI dvec2 normalize(const dvec2& v);

CybertronCoreAPI double length(const dvec2& v);

CybertronCoreAPI double distance(const dvec2& v1, const dvec2& v2);

CybertronCoreAPI double dot(const dvec2& lhs, const dvec2& rhs);

CYBERTRON_END
