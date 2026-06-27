// 
// Created By: Qilin.Ren 2018/05/29
// 

#pragma once

#include "../DefinesCore.hpp"

CYBERTRON_BEGIN

struct dmat4
{
public:
	double m[4][4];

	CybertronCoreAPI dmat4();

	CybertronCoreAPI explicit dmat4(const mat4& rhs);

	CybertronCoreAPI explicit dmat4(const dmat3& rhs);

	CybertronCoreAPI explicit dmat4(double s);

	CybertronCoreAPI dmat4(const dmat4& rhs);

	CybertronCoreAPI dmat4(
		const dvec4& val1,
		const dvec4& val2,
		const dvec4& val3,
		const dvec4& val4);

	CybertronCoreAPI dmat4(
		double x0, double y0, double z0, double w0,
		double x1, double y1, double z1, double w1,
		double x2, double y2, double z2, double w2,
		double x3, double y3, double z3, double w3);

	~dmat4()
	{
	}

	CybertronCoreAPI bool operator == (const dmat4& rhs) const;

	bool operator != (const dmat4& rhs) const
	{
		return !(*this == rhs);
	}

	CybertronCoreAPI dvec4& operator[](int col);

	CybertronCoreAPI const dvec4& operator[](int col) const;

	CybertronCoreAPI dmat4 operator * (const dmat4& rhs) const;

	CybertronCoreAPI dmat4 operator * (double val) const;

	CybertronCoreAPI dvec4 operator * (const dvec4& rowVec) const;

	CybertronCoreAPI dvec3 operator * (const dvec3& rowVec) const;

	CybertronCoreAPI dmat4 operator + (const dmat4& rhs) const;

	CybertronCoreAPI dmat4 operator - (const dmat4& rhs) const;

	CybertronCoreAPI dmat4 operator -() const;
};

inline double* value_ptr(dmat4& m)
{
	return (double*)(m.m);
}

inline const double* value_ptr(const dmat4& m)
{
	return (const double*)(m.m);
}

CybertronCoreAPI dvec4 operator * (const dvec4& colVec, const dmat4& m);

CybertronCoreAPI dmat4 translate(const dvec3& position);

CybertronCoreAPI dmat4 scale(const dvec3& scale);

CybertronCoreAPI dmat4 transpose(const dmat4& m);

CybertronCoreAPI dmat4 perspective(
	double fov,
	double aspectRatio,
	double zNear,
	double zFar);

CybertronCoreAPI dmat4 ortho(
	double left,
	double right,
	double bottom,
	double top,
	double zNear,
	double zFar);

CybertronCoreAPI dmat4 inverse(const dmat4& m);

CybertronCoreAPI dmat4 mix(const dmat4& val1, const dmat4& val2, double a);

CybertronCoreAPI dmat4 lookAt(const dvec3& eye, const dvec3& lookAt, const dvec3& up);

CybertronCoreAPI double determinant(const dmat4& m);

CybertronCoreAPI bool decompose(
	const dmat4& m,
	dvec3& scale,
	dquat& rotation,
	dvec3& translation,
	dvec3& skew,
	dvec4& perspective);

CybertronCoreAPI dmat4 rotate(double angle, const dvec3& dir);

CybertronCoreAPI dmat4 matrixCompMult(const dmat4& lhs, const dmat4& rhs);

CYBERTRON_END
