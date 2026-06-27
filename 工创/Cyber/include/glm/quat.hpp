// 
// Created By: Qilin.Ren 2018/05/29
// 

#pragma once

#include "../DefinesCore.hpp"
#ifndef CYBERTRON_WIN
#include "stdlib.h"
#endif

CYBERTRON_BEGIN

struct quat
{
public:
	float x, y, z, w;

	quat()
	{
	}

	CybertronCoreAPI explicit quat(const mat3& m);

	CybertronCoreAPI explicit quat(const mat4& m);

	CybertronCoreAPI quat(const vec3& from, const vec3& to);

	CybertronCoreAPI quat(const vec3& eulerAngles);

    CybertronCoreAPI quat(float angleRadians, const vec3& unitAxis);

	quat(float ww, float xx, float yy, float zz)
	{
		x = xx;
		y = yy;
		z = zz;
		w = ww;
	}

    quat(float r) : x(0.0f), y(0.0f), z(0.0f), w(r)
    {
    }

	quat(const quat& rhs)
	{
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		w = rhs.w;
	}

	~quat()
	{
	}

	bool operator == (const quat& rhs) const
	{
		return x == rhs.x && y == rhs.y && z == rhs.z && w == rhs.w;
	}

	bool operator != (const quat& rhs) const
	{
		return !(*this == rhs);
	}

	float& operator[](size_t index)
	{
		return (&x)[index];
	}

	float operator[](size_t index) const
	{
		return (&x)[index];
	}

	CybertronCoreAPI quat operator * (const quat& rhs) const;

	CybertronCoreAPI quat operator * (float rhs) const;

	CybertronCoreAPI quat operator / (float rhs) const;

	CybertronCoreAPI quat operator + (const quat& rhs) const;

	CybertronCoreAPI quat operator - (const quat& rhs) const;

	CybertronCoreAPI quat operator - () const;

	CybertronCoreAPI quat& operator *= (const quat& rhs);

	CybertronCoreAPI vec3 operator * (const vec3& rhs) const;
};

inline float* value_ptr(quat& m)
{
	return (float*)(&m.x);
}

inline const float* value_ptr(const quat& m)
{
	return (const float*)(&m.x);
}

CybertronCoreAPI quat operator * (float lhs, const quat& rhs);

CybertronCoreAPI float dot(const quat& q1, const quat& q2);

CybertronCoreAPI float length(const quat& q);

CybertronCoreAPI quat quat_cast(const mat3& m);

CybertronCoreAPI quat quat_cast(const mat4& m);

CybertronCoreAPI mat3 mat3_cast(const quat& q);

CybertronCoreAPI mat4 mat4_cast(const quat& q);

CybertronCoreAPI quat lerp(const quat& q1, const quat& q2, float a);

CybertronCoreAPI quat slerp(const quat& q1, const quat& q2, float a);

CybertronCoreAPI vec3 eulerAngles(const quat& q);

CybertronCoreAPI mat4 eulerAngleXYZ(float x, float y, float z);

CybertronCoreAPI quat normalize(const quat& q);

CybertronCoreAPI quat conjugate(const quat& q);

CybertronCoreAPI quat inverse(const quat& q);

CybertronCoreAPI quat cross(const quat& q1, const quat& q2);

CybertronCoreAPI quat angleAxis(float angle, const vec3& dir);

CybertronCoreAPI vec3 rotate(const quat& q, const vec3& v);

CybertronCoreAPI bool isFinite(const quat& q);

/**
\brief returns true if finite and magnitude is close to unit
*/
CybertronCoreAPI bool isUnit(const quat& q);

/**
\brief returns true if finite and magnitude is reasonably close to unit to allow for some accumulation of error vs
isValid
*/
CybertronCoreAPI bool isSane(const quat& q);

CybertronCoreAPI const vec3 rotateInv(const quat& q, const vec3& v);

CYBERTRON_END
