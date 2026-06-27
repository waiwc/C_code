// 
// Created By: Qilin.Ren 2018/05/29
// 

#pragma once

#include "../DefinesCore.hpp"

CYBERTRON_BEGIN

struct quat;

struct mat3
{
public:
	float m[3][3];

	CybertronCoreAPI mat3();

	CybertronCoreAPI explicit mat3(const mat4& m);

	CybertronCoreAPI explicit mat3(float s);

	CybertronCoreAPI mat3(const mat3& rhs);

	CybertronCoreAPI mat3(
		float x0, float y0, float z0,
		float x1, float y1, float z1,
		float x2, float y2, float z2);

    CybertronCoreAPI explicit mat3(const quat& q);

	~mat3()
	{
	}

	CybertronCoreAPI bool operator == (const mat3& rhs) const;

	bool operator != (const mat3& rhs) const
	{
		return !(*this == rhs);
	}

	CybertronCoreAPI vec3& operator[](int col);

	CybertronCoreAPI const vec3& operator[](int col) const;

	CybertronCoreAPI mat3 operator * (const mat3& rhs) const;

	CybertronCoreAPI mat3& operator *= (const mat3& rhs);

	CybertronCoreAPI vec3 operator * (const vec3& rhs) const;
};

inline float* value_ptr(mat3& m)
{
	return (float*)(m.m);
}

inline const float* value_ptr(const mat3& m)
{
	return (const float*)(m.m);
}

CybertronCoreAPI mat3 operator * (float lhs, const mat3& rhs);

CybertronCoreAPI float determinant(const mat3& m);

CybertronCoreAPI mat3 inverse(const mat3& m);
CYBERTRON_END
