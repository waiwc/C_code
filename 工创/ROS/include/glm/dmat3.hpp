// 
// Created By: Qilin.Ren 2018/05/29
// 

#pragma once

#include "../DefinesCore.hpp"

CYBERTRON_BEGIN

struct dmat3
{
public:
	double m[3][3];

	CybertronCoreAPI dmat3();

	CybertronCoreAPI explicit dmat3(const dmat4& m);

	CybertronCoreAPI explicit dmat3(double s);

	CybertronCoreAPI dmat3(
		double x0, double y0, double z0,
		double x1, double y1, double z1,
		double x2, double y2, double z2);

	CybertronCoreAPI dmat3(const dmat3& rhs);

	~dmat3()
	{
	}

	CybertronCoreAPI bool operator == (const dmat3& rhs) const;

	bool operator != (const dmat3& rhs) const
	{
		return !(*this == rhs);
	}

	CybertronCoreAPI dvec3& operator[](int col);

	CybertronCoreAPI const dvec3& operator[](int col) const;

	CybertronCoreAPI dmat3 operator * (const dmat3& rhs) const;

	CybertronCoreAPI dmat3& operator *= (const dmat3& rhs);

	CybertronCoreAPI dvec3 operator * (const dvec3& rhs) const;
};

inline double* value_ptr(dmat3& m)
{
	return (double*)(m.m);
}

inline const double* value_ptr(const dmat3& m)
{
	return (const double*)(m.m);
}

CybertronCoreAPI dmat3 operator * (double lhs, const dmat3& rhs);

CybertronCoreAPI double determinant(const dmat3& m);

CybertronCoreAPI dmat3 inverse(const dmat3& m);

CYBERTRON_END
