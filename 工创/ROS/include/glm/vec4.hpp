// 
// Created By: Qilin.Ren 2018/05/29
// 

#pragma once

#include "../DefinesCore.hpp"

CYBERTRON_BEGIN

struct vec4
{
public:
	float x, y, z, w;

	vec4()
	{
	}

	explicit vec4(float f)
	{
		x = f;
		y = f;
		z = f;
		w = f;
	}

	vec4(float xx, float yy, float zz, float ww)
	{
		x = xx;
		y = yy;
		z = zz;
		w = ww;
	}

	CybertronCoreAPI vec4(const vec3& rhs, float ww);

	vec4(const vec4& rhs)
	{
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		w = rhs.w;
	}

	~vec4()
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

	bool operator == (const vec4& rhs) const
	{
		return x == rhs.x && y == rhs.y && z == rhs.z && w == rhs.w;
	}

	bool operator != (const vec4& rhs) const
	{
		return !(*this == rhs);
	}

	vec4 operator + (const vec4& rhs) const
	{
		return vec4(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w);
	}

	vec4 operator - (const vec4& rhs) const
	{
		return vec4(x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w);
	}

	vec4 operator + (float rhs) const
	{
		return vec4(x + rhs, y + rhs, z + rhs, w + rhs);
	}

	vec4 operator - (float rhs) const
	{
		return vec4(x - rhs, y - rhs, z - rhs, w - rhs);
	}

	vec4 operator * (float f) const
	{
		return vec4(x * f, y * f, z * f, w * f);
	}

	vec4 operator / (float f) const
	{
		return vec4(x / f, y / f, z / f, w / f);
	}

	vec4 operator * (const vec4& v) const
	{
		return vec4(x * v.x, y * v.y, z * v.z, w * v.w);
	}

	vec4 operator / (const vec4& v) const
	{
		return vec4(x / v.x, y / v.y, z / v.z, w / v.w);
	}

	vec4& operator += (const vec4& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
		w += rhs.w;
		return *this;
	}

	vec4& operator -= (const vec4& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
		w -= rhs.w;
		return *this;
	}

	vec4& operator *= (float rhs)
	{
		x *= rhs;
		y *= rhs;
		z *= rhs;
		w *= rhs;
		return *this;
	}

	vec4& operator /= (float rhs)
	{
		x /= rhs;
		y /= rhs;
		z /= rhs;
		w /= rhs;
		return *this;
	}

	CybertronCoreAPI vec3 xyz() const;

	vec4 operator -() const
	{
		return vec4(-x, -y, -z, -w);
	}
};

inline float* value_ptr(vec4& m)
{
	return (float*)(&m.x);
}

inline const float* value_ptr(const vec4& m)
{
	return (const float*)(&m.x);
}

inline vec4 make_vec4(const float* pValues)
{
	return vec4(pValues[0], pValues[1], pValues[2], pValues[3]);
}

CybertronCoreAPI vec4 operator * (float lhs, const vec4& rhs);

CybertronCoreAPI vec4 normalize(const vec4& v);

CYBERTRON_END
