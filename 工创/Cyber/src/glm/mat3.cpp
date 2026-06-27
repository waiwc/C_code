// 
// Created By: Qilin.Ren 2018/05/29
// 

#include "glm/mat3.hpp"
#include "glm/mat4.hpp"
#include "glm/vec3.hpp"
#include "glm/quat.hpp"

CYBERTRON_BEGIN

CybertronCoreAPI mat3::mat3()
{
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			m[i][j] = 0;
		}
	}
}

CybertronCoreAPI mat3::mat3(const mat4& rhs)
{
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			m[i][j] = rhs.m[i][j];
		}
	}
}

CybertronCoreAPI mat3::mat3(float s)
{
	m[0][0] = s;
	m[0][1] = 0;
	m[0][2] = 0;

	m[1][0] = 0;
	m[1][1] = s;
	m[1][2] = 0;

	m[2][0] = 0;
	m[2][1] = 0;
	m[2][2] = s;
}

CybertronCoreAPI mat3::mat3(
	float x0, float y0, float z0,
	float x1, float y1, float z1,
	float x2, float y2, float z2)
{
	m[0][0] = x0;
	m[0][1] = y0;
	m[0][2] = z0;

	m[1][0] = x1;
	m[1][1] = y1;
	m[1][2] = z1;

	m[2][0] = x2;
	m[2][1] = y2;
	m[2][2] = z2;
}

CybertronCoreAPI mat3::mat3(const mat3& rhs)
{
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			m[i][j] = rhs.m[i][j];
		}
	}
}

CybertronCoreAPI mat3::mat3(const quat& q)
{
    const float x = q.x;
    const float y = q.y;
    const float z = q.z;
    const float w = q.w;

    const float x2 = x + x;
    const float y2 = y + y;
    const float z2 = z + z;

    const float xx = x2 * x;
    const float yy = y2 * y;
    const float zz = z2 * z;

    const float xy = x2 * y;
    const float xz = x2 * z;
    const float xw = x2 * w;

    const float yz = y2 * z;
    const float yw = y2 * w;
    const float zw = z2 * w;

    m[0][0] = 1.0f - yy - zz;
    m[0][1] = xy + zw;
    m[0][2] = xz - yw;
    m[1][0] = xy - zw;
    m[1][1] = 1.0f - xx - zz;
    m[1][2] = yz + xw;
    m[2][0] = xz + yw;
    m[2][1] = yz - xw;
    m[2][2] = 1.0f - xx - yy;
}

CybertronCoreAPI mat3 operator * (float lhs, const mat3& rhs)
{
	mat3 result;
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			result.m[i][j] = lhs * rhs.m[i][j];
		}
	}

	return result;
}

CybertronCoreAPI bool mat3::operator == (const mat3& rhs) const
{
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (m[i][j] != rhs.m[i][j])
			{
				return false;
			}
		}
	}

	return true;
}

CybertronCoreAPI vec3& mat3::operator[](int col)
{
	float* pAddr = m[col];
	return *((vec3*)pAddr);
}

CybertronCoreAPI const vec3& mat3::operator[](int col) const
{
	const float* pAddr = m[col];
	return *((vec3*)pAddr);
}

CybertronCoreAPI mat3 mat3::operator * (const mat3& m2) const
{
	const mat3& m1 = (*this);

	float const SrcA00 = m1[0][0];
	float const SrcA01 = m1[0][1];
	float const SrcA02 = m1[0][2];
	float const SrcA10 = m1[1][0];
	float const SrcA11 = m1[1][1];
	float const SrcA12 = m1[1][2];
	float const SrcA20 = m1[2][0];
	float const SrcA21 = m1[2][1];
	float const SrcA22 = m1[2][2];

	float const SrcB00 = m2[0][0];
	float const SrcB01 = m2[0][1];
	float const SrcB02 = m2[0][2];
	float const SrcB10 = m2[1][0];
	float const SrcB11 = m2[1][1];
	float const SrcB12 = m2[1][2];
	float const SrcB20 = m2[2][0];
	float const SrcB21 = m2[2][1];
	float const SrcB22 = m2[2][2];

	mat3 Result;
	Result[0][0] = SrcA00 * SrcB00 + SrcA10 * SrcB01 + SrcA20 * SrcB02;
	Result[0][1] = SrcA01 * SrcB00 + SrcA11 * SrcB01 + SrcA21 * SrcB02;
	Result[0][2] = SrcA02 * SrcB00 + SrcA12 * SrcB01 + SrcA22 * SrcB02;
	Result[1][0] = SrcA00 * SrcB10 + SrcA10 * SrcB11 + SrcA20 * SrcB12;
	Result[1][1] = SrcA01 * SrcB10 + SrcA11 * SrcB11 + SrcA21 * SrcB12;
	Result[1][2] = SrcA02 * SrcB10 + SrcA12 * SrcB11 + SrcA22 * SrcB12;
	Result[2][0] = SrcA00 * SrcB20 + SrcA10 * SrcB21 + SrcA20 * SrcB22;
	Result[2][1] = SrcA01 * SrcB20 + SrcA11 * SrcB21 + SrcA21 * SrcB22;
	Result[2][2] = SrcA02 * SrcB20 + SrcA12 * SrcB21 + SrcA22 * SrcB22;

	return Result;
}

CybertronCoreAPI mat3& mat3::operator *= (const mat3& m2)
{
	mat3& m1 = (*this);

	float const SrcA00 = m1[0][0];
	float const SrcA01 = m1[0][1];
	float const SrcA02 = m1[0][2];
	float const SrcA10 = m1[1][0];
	float const SrcA11 = m1[1][1];
	float const SrcA12 = m1[1][2];
	float const SrcA20 = m1[2][0];
	float const SrcA21 = m1[2][1];
	float const SrcA22 = m1[2][2];

	float const SrcB00 = m2[0][0];
	float const SrcB01 = m2[0][1];
	float const SrcB02 = m2[0][2];
	float const SrcB10 = m2[1][0];
	float const SrcB11 = m2[1][1];
	float const SrcB12 = m2[1][2];
	float const SrcB20 = m2[2][0];
	float const SrcB21 = m2[2][1];
	float const SrcB22 = m2[2][2];

	m1[0][0] = SrcA00 * SrcB00 + SrcA10 * SrcB01 + SrcA20 * SrcB02;
	m1[0][1] = SrcA01 * SrcB00 + SrcA11 * SrcB01 + SrcA21 * SrcB02;
	m1[0][2] = SrcA02 * SrcB00 + SrcA12 * SrcB01 + SrcA22 * SrcB02;
	m1[1][0] = SrcA00 * SrcB10 + SrcA10 * SrcB11 + SrcA20 * SrcB12;
	m1[1][1] = SrcA01 * SrcB10 + SrcA11 * SrcB11 + SrcA21 * SrcB12;
	m1[1][2] = SrcA02 * SrcB10 + SrcA12 * SrcB11 + SrcA22 * SrcB12;
	m1[2][0] = SrcA00 * SrcB20 + SrcA10 * SrcB21 + SrcA20 * SrcB22;
	m1[2][1] = SrcA01 * SrcB20 + SrcA11 * SrcB21 + SrcA21 * SrcB22;
	m1[2][2] = SrcA02 * SrcB20 + SrcA12 * SrcB21 + SrcA22 * SrcB22;

	return *this;
}

CybertronCoreAPI vec3 mat3::operator * (const vec3& v) const
{
	return vec3(
		m[0][0] * v.x + m[1][0] * v.y + m[2][0] * v.z,
		m[0][1] * v.x + m[1][1] * v.y + m[2][1] * v.z,
		m[0][2] * v.x + m[1][2] * v.y + m[2][2] * v.z);
}

CybertronCoreAPI float determinant(const mat3& m)
{
	return
		+m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2])
		- m[1][0] * (m[0][1] * m[2][2] - m[2][1] * m[0][2])
		+ m[2][0] * (m[0][1] * m[1][2] - m[1][1] * m[0][2]);
}

CybertronCoreAPI mat3 inverse(const mat3& m)
{
	float OneOverDeterminant = 1 / determinant(m);

	mat3 Inverse;
	Inverse[0][0] = +(m[1][1] * m[2][2] - m[2][1] * m[1][2]) * OneOverDeterminant;
	Inverse[1][0] = -(m[1][0] * m[2][2] - m[2][0] * m[1][2]) * OneOverDeterminant;
	Inverse[2][0] = +(m[1][0] * m[2][1] - m[2][0] * m[1][1]) * OneOverDeterminant;
	Inverse[0][1] = -(m[0][1] * m[2][2] - m[2][1] * m[0][2]) * OneOverDeterminant;
	Inverse[1][1] = +(m[0][0] * m[2][2] - m[2][0] * m[0][2]) * OneOverDeterminant;
	Inverse[2][1] = -(m[0][0] * m[2][1] - m[2][0] * m[0][1]) * OneOverDeterminant;
	Inverse[0][2] = +(m[0][1] * m[1][2] - m[1][1] * m[0][2]) * OneOverDeterminant;
	Inverse[1][2] = -(m[0][0] * m[1][2] - m[1][0] * m[0][2]) * OneOverDeterminant;
	Inverse[2][2] = +(m[0][0] * m[1][1] - m[1][0] * m[0][1]) * OneOverDeterminant;

	return Inverse;
}

CYBERTRON_END
