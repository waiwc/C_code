// 
// Created By: Qilin.Ren 2018/05/29
// 

#pragma once

#include "vec2.hpp"
#include "bvec3.hpp"
#ifndef CYBERTRON_WIN
#include "stdlib.h"
#endif

CYBERTRON_BEGIN

struct vec3
{
public:
	float x, y, z;

	vec3()
	{
	}

	vec3(const vec2& xy, float zz)
	{
		x = xy.x;
		y = xy.y;
		z = zz;
	}

	explicit vec3(float f)
	{
		x = f;
		y = f;
		z = f;
	}

	vec3(float xx, float yy, float zz)
	{
		x = xx;
		y = yy;
		z = zz;
	}

	vec3(const vec3& rhs)
	{
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
	}

	vec3(const float* p)
	{
		x = p[0];
		y = p[1];
		z = p[2];
	}

	vec3(const double* p)
	{
		x = (float)p[0];
		y = (float)p[1];
		z = (float)p[2];
	}

	CybertronCoreAPI vec3(const vec4& rhs);

	~vec3()
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

	bool operator == (const vec3& rhs) const
	{
		return x == rhs.x && y == rhs.y && z == rhs.z;
	}

	bool operator != (const vec3& rhs) const
	{
		return !(*this == rhs);
	}

	vec3 operator + (const vec3& rhs) const
	{
		return vec3(x + rhs.x, y + rhs.y, z + rhs.z);
	}

	vec3 operator - (const vec3& rhs) const
	{
		return vec3(x - rhs.x, y - rhs.y, z - rhs.z);
	}

	vec3 operator + (float rhs) const
	{
		return vec3(x + rhs, y + rhs, z + rhs);
	}

	vec3 operator - (float rhs) const
	{
		return vec3(x - rhs, y - rhs, z - rhs);
	}

	vec3 operator * (float f) const
	{
		return vec3(x * f, y * f, z * f);
	}

	vec3 operator / (float f) const
	{
		return vec3(x / f, y / f, z / f);
	}

	vec3 operator * (const vec3& rhs) const
	{
		return vec3(x * rhs.x, y * rhs.y, z * rhs.z);
	}

	vec3 operator / (const vec3& rhs) const
	{
		return vec3(x / rhs.x, y / rhs.y, z / rhs.z);
	}

	vec3 operator -() const
	{
		return vec3(-x, -y, -z);
	}

	vec3& operator += (const vec3& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;

		return *this;
	}

	vec3& operator -= (const vec3& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;

		return *this;
	}

	vec3& operator *= (const vec3& rhs)
	{
		x *= rhs.x;
		y *= rhs.y;
		z *= rhs.z;

		return *this;
	}

	vec3& operator /= (const vec3& rhs)
	{
		x /= rhs.x;
		y /= rhs.y;
		z /= rhs.z;

		return *this;
	}

	vec3& operator *= (float rhs)
	{
		x *= rhs;
		y *= rhs;
		z *= rhs;

		return *this;
	}

	vec3& operator /= (float rhs)
	{
		x /= rhs;
		y /= rhs;
		z /= rhs;

		return *this;
	}
};

inline float* value_ptr(vec3& m)
{
	return (float*)(&m.x);
}

inline const float* value_ptr(const vec3& m)
{
	return (const float*)(&m.x);
}

inline vec3 make_vec3(const float* pValues)
{
	return vec3(pValues[0], pValues[1], pValues[2]);
}

inline vec3 mix(const vec3& val1, const vec3& val2, float a)
{
	return val1 * (1 - a) + val2 * a;
}

CybertronCoreAPI vec3 min(const vec3& lhs, const vec3& rhs);

CybertronCoreAPI vec3 max(const vec3& lhs, const vec3& rhs);

CybertronCoreAPI vec3 operator * (float lhs, const vec3& rhs);

CybertronCoreAPI vec3 operator / (float lhs, const vec3& rhs);

CybertronCoreAPI vec3 cross(const vec3& lhs, const vec3& rhs);

CybertronCoreAPI float dot(const vec3& lhs, const vec3& rhs);

CybertronCoreAPI vec3 normalize(const vec3& v);

CybertronCoreAPI float length(const vec3& v);

CybertronCoreAPI float distance(const vec3& v1, const vec3& v2);

CybertronCoreAPI bvec3 equal(const vec3& lhs, const vec3& rhs);

CYBERTRON_END
