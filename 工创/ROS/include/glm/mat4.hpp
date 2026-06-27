// 
// Created By: Qilin.Ren 2018/05/29
// 

#pragma once

#include "../DefinesCore.hpp"

CYBERTRON_BEGIN

struct mat4
{
public:
	float m[4][4];

	CybertronCoreAPI mat4();

	CybertronCoreAPI explicit mat4(const mat3& rhs);

	CybertronCoreAPI explicit mat4(float s);

	CybertronCoreAPI mat4(const mat4& rhs);

	CybertronCoreAPI mat4(
		const vec4& val1,
		const vec4& val2,
		const vec4& val3,
		const vec4& val4);

	CybertronCoreAPI mat4(
		float x0, float y0, float z0, float w0,
		float x1, float y1, float z1, float w1,
		float x2, float y2, float z2, float w2,
		float x3, float y3, float z3, float w3);

	~mat4()
	{
	}

	CybertronCoreAPI bool operator == (const mat4& rhs) const;

	bool operator != (const mat4& rhs) const
	{
		return !(*this == rhs);
	}

	CybertronCoreAPI vec4& operator[](int col);

	CybertronCoreAPI const vec4& operator[](int col) const;

	CybertronCoreAPI mat4 operator * (const mat4& rhs) const;

	CybertronCoreAPI mat4 operator * (float val) const;

	CybertronCoreAPI vec4 operator * (const vec4& rowVec) const;

	CybertronCoreAPI vec3 operator * (const vec3& rowVec) const;

	CybertronCoreAPI mat4 operator + (const mat4& rhs) const;

	CybertronCoreAPI mat4 operator - (const mat4& rhs) const;

	CybertronCoreAPI mat4 operator -() const;
};

inline float* value_ptr(mat4& m)
{
	return (float*)(m.m);
}

inline const float* value_ptr(const mat4& m)
{
	return (const float*)(m.m);
}

CybertronCoreAPI vec4 operator * (const vec4& colVec, const mat4& m);

CybertronCoreAPI mat4 translate(const vec3& position);

CybertronCoreAPI mat4 scale(const vec3& scale);

CybertronCoreAPI mat4 transpose(const mat4& m);

CybertronCoreAPI mat4 perspective(
	float fov,
	float aspectRatio,
	float zNear,
	float zFar);

CybertronCoreAPI mat4 ortho(
	float left,
	float right,
	float bottom,
	float top,
	float zNear,
	float zFar);

CybertronCoreAPI mat4 inverse(const mat4& m);

CybertronCoreAPI mat4 mix(const mat4& val1, const mat4& val2, float a);

CybertronCoreAPI mat4 lookAt(const vec3& eye, const vec3& lookAt, const vec3& up);

CybertronCoreAPI float determinant(const mat4& m);

CybertronCoreAPI bool decompose(
	const mat4& m,
	vec3& scale,
	quat& rotation,
	vec3& translation,
	vec3& skew,
	vec4& perspective);

CybertronCoreAPI mat4 rotate(float angle, const vec3& dir);

CybertronCoreAPI mat4 matrixCompMult(const mat4& lhs, const mat4& rhs);

CybertronCoreAPI mat4 eulerAngleX(float angleX);

CybertronCoreAPI mat4 eulerAngleY(float angleX);

CybertronCoreAPI mat4 eulerAngleZ(float angleX);

CYBERTRON_END
