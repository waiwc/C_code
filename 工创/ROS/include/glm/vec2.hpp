// 
// Created By: Qilin.Ren 2018/05/29
// 

#pragma once

#include "../DefinesCore.hpp"
#ifndef CYBERTRON_WIN
#include "stdlib.h"
#endif

CYBERTRON_BEGIN

struct vec2
{
public:
	float x, y;

	vec2()
	{
	}

	explicit vec2(float f)
	{
		x = f;
		y = f;
	}

	vec2(float xx, float yy)
	{
		x = xx;
		y = yy;
	}

	vec2(const vec2& rhs)
	{
		x = rhs.x;
		y = rhs.y;
	}

	~vec2()
	{
	}

	float& operator[](size_t index)
	{
		return (&x)[index];
	}

	float operator[](size_t index) const
	{
		return (&x)[index];
	}

	bool operator == (const vec2& rhs) const
	{
		return x == rhs.x && y == rhs.y;
	}

	bool operator != (const vec2& rhs) const
	{
		return !(*this == rhs);
	}

	vec2 operator - (const vec2& rhs) const
	{
		return vec2(x - rhs.x, y - rhs.y);
	}

	vec2 operator + (const vec2& rhs) const
	{
		return vec2(x + rhs.x, y + rhs.y);
	}

	vec2 operator * (float f) const
	{
		return vec2(x * f, y * f);
	}

	vec2 operator / (float f) const
	{
		return vec2(x / f, y / f);
	}

	vec2& operator += (const vec2& rhs)
	{
		x += rhs.x;
		y += rhs.y;

		return *this;
	}

	vec2& operator -= (const vec2& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;

		return *this;
	}

	vec2& operator *= (const vec2& rhs)
	{
		x *= rhs.x;
		y *= rhs.y;

		return *this;
	}

	vec2& operator /= (const vec2& rhs)
	{
		x /= rhs.x;
		y /= rhs.y;

		return *this;
	}

	vec2& operator *= (float rhs)
	{
		x *= rhs;
		y *= rhs;

		return *this;
	}

	vec2& operator /= (float rhs)
	{
		x /= rhs;
		y /= rhs;

		return *this;
	}
};

inline vec2 operator * (float lhs, const vec2& rhs)
{
	return vec2(lhs * rhs.x, lhs * rhs.y);
}

inline float* value_ptr(vec2& m)
{
	return (float*)(&m.x);
}

inline const float* value_ptr(const vec2& m)
{
	return (const float*)(&m.x);
}

inline vec2 make_vec2(const float* pValues)
{
	return vec2(pValues[0], pValues[1]);
}

inline vec2 mix(const vec2& val1, const vec2& val2, float a)
{
	return val1 * (1 - a) + val2 * a;
}

CybertronCoreAPI vec2 normalize(const vec2& v);

CybertronCoreAPI float length(const vec2& v);

CybertronCoreAPI double distance(const vec2& v1, const vec2& v2);

CybertronCoreAPI float dot(const vec2& lhs, const vec2& rhs);

CYBERTRON_END
