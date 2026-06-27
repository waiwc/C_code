// 
// Created By: Qilin.Ren 2018/05/29
// 

#pragma once

#include "../DefinesCore.hpp"

CYBERTRON_BEGIN

struct dvec4
{
public:
	double x, y, z, w;

	dvec4()
	{
	}

	explicit dvec4(double f)
	{
		x = f;
		y = f;
		z = f;
		w = f;
	}

	dvec4(double xx, double yy, double zz, double ww)
	{
		x = xx;
		y = yy;
		z = zz;
		w = ww;
	}

	CybertronCoreAPI dvec4(const dvec3& rhs, double ww);

	dvec4(const dvec4& rhs)
	{
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		w = rhs.w;
	}

	~dvec4()
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

	bool operator == (const dvec4& rhs) const
	{
		return x == rhs.x && y == rhs.y && z == rhs.z && w == rhs.w;
	}

	bool operator != (const dvec4& rhs) const
	{
		return !(*this == rhs);
	}

	dvec4 operator + (const dvec4& rhs) const
	{
		return dvec4(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w);
	}

	dvec4 operator - (const dvec4& rhs) const
	{
		return dvec4(x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w);
	}

	dvec4 operator + (double rhs) const
	{
		return dvec4(x + rhs, y + rhs, z + rhs, w + rhs);
	}

	dvec4 operator - (double rhs) const
	{
		return dvec4(x - rhs, y - rhs, z - rhs, w - rhs);
	}

	dvec4 operator * (double f) const
	{
		return dvec4(x * f, y * f, z * f, w * f);
	}

	dvec4 operator / (double f) const
	{
		return dvec4(x / f, y / f, z / f, w / f);
	}

	dvec4 operator * (const dvec4& v) const
	{
		return dvec4(x * v.x, y * v.y, z * v.z, w * v.w);
	}

	dvec4 operator / (const dvec4& v) const
	{
		return dvec4(x / v.x, y / v.y, z / v.z, w / v.w);
	}

	dvec4& operator += (const dvec4& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
		w += rhs.w;
		return *this;
	}

	dvec4& operator -= (const dvec4& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
		w -= rhs.w;
		return *this;
	}

	dvec4& operator *= (double rhs)
	{
		x *= rhs;
		y *= rhs;
		z *= rhs;
		w *= rhs;
		return *this;
	}

	dvec4& operator /= (double rhs)
	{
		x /= rhs;
		y /= rhs;
		z /= rhs;
		w /= rhs;
		return *this;
	}

	CybertronCoreAPI dvec3 xyz() const;

	dvec4 operator -() const
	{
		return dvec4(-x, -y, -z, -w);
	}
};

inline double* value_ptr(dvec4& m)
{
	return (double*)(&m.x);
}

inline const double* value_ptr(const dvec4& m)
{
	return (const double*)(&m.x);
}

inline dvec4 make_vec4(const double* pValues)
{
	return dvec4(pValues[0], pValues[1], pValues[2], pValues[3]);
}

CybertronCoreAPI dvec4 operator * (double lhs, const dvec4& rhs);

CybertronCoreAPI dvec4 normalize(const dvec4& v);

CYBERTRON_END
