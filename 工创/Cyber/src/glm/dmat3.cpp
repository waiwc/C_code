// 
// Created By: Qilin.Ren 2018/05/29
// 

#include "glm/dmat3.hpp"
#include "glm/dmat4.hpp"
#include "glm/dvec3.hpp"

CYBERTRON_BEGIN

CybertronCoreAPI dmat3::dmat3()
{
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			m[i][j] = 0;
		}
	}
}

CybertronCoreAPI dmat3::dmat3(const dmat4& rhs)
{
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			m[i][j] = rhs.m[i][j];
		}
	}
}

CybertronCoreAPI dmat3::dmat3(double s)
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

CybertronCoreAPI dmat3::dmat3(
	double x0, double y0, double z0,
	double x1, double y1, double z1,
	double x2, double y2, double z2)
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

CybertronCoreAPI dmat3::dmat3(const dmat3& rhs)
{
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			m[i][j] = rhs.m[i][j];
		}
	}
}

CybertronCoreAPI dmat3 operator * (double lhs, const dmat3& rhs)
{
	dmat3 result;
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			result.m[i][j] = lhs * rhs.m[i][j];
		}
	}

	return result;
}

CybertronCoreAPI bool dmat3::operator == (const dmat3& rhs) const
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

CybertronCoreAPI dvec3& dmat3::operator[](int col)
{
	double* pAddr = m[col];
	return *((dvec3*)pAddr);
}

CybertronCoreAPI const dvec3& dmat3::operator[](int col) const
{
	const double* pAddr = m[col];
	return *((dvec3*)pAddr);
}

CybertronCoreAPI dmat3 dmat3::operator * (const dmat3& m2) const
{
	const dmat3& m1 = (*this);

	double const SrcA00 = m1[0][0];
	double const SrcA01 = m1[0][1];
	double const SrcA02 = m1[0][2];
	double const SrcA10 = m1[1][0];
	double const SrcA11 = m1[1][1];
	double const SrcA12 = m1[1][2];
	double const SrcA20 = m1[2][0];
	double const SrcA21 = m1[2][1];
	double const SrcA22 = m1[2][2];

	double const SrcB00 = m2[0][0];
	double const SrcB01 = m2[0][1];
	double const SrcB02 = m2[0][2];
	double const SrcB10 = m2[1][0];
	double const SrcB11 = m2[1][1];
	double const SrcB12 = m2[1][2];
	double const SrcB20 = m2[2][0];
	double const SrcB21 = m2[2][1];
	double const SrcB22 = m2[2][2];

	dmat3 Result;
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

CybertronCoreAPI dmat3& dmat3::operator *= (const dmat3& m2)
{
	dmat3& m1 = (*this);

	double const SrcA00 = m1[0][0];
	double const SrcA01 = m1[0][1];
	double const SrcA02 = m1[0][2];
	double const SrcA10 = m1[1][0];
	double const SrcA11 = m1[1][1];
	double const SrcA12 = m1[1][2];
	double const SrcA20 = m1[2][0];
	double const SrcA21 = m1[2][1];
	double const SrcA22 = m1[2][2];

	double const SrcB00 = m2[0][0];
	double const SrcB01 = m2[0][1];
	double const SrcB02 = m2[0][2];
	double const SrcB10 = m2[1][0];
	double const SrcB11 = m2[1][1];
	double const SrcB12 = m2[1][2];
	double const SrcB20 = m2[2][0];
	double const SrcB21 = m2[2][1];
	double const SrcB22 = m2[2][2];

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

CybertronCoreAPI dvec3 dmat3::operator * (const dvec3& v) const
{
	return dvec3(
		m[0][0] * v.x + m[1][0] * v.y + m[2][0] * v.z,
		m[0][1] * v.x + m[1][1] * v.y + m[2][1] * v.z,
		m[0][2] * v.x + m[1][2] * v.y + m[2][2] * v.z);
}

CybertronCoreAPI double determinant(const dmat3& m)
{
	return
		+m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2])
		- m[1][0] * (m[0][1] * m[2][2] - m[2][1] * m[0][2])
		+ m[2][0] * (m[0][1] * m[1][2] - m[1][1] * m[0][2]);
}

CybertronCoreAPI dmat3 inverse(const dmat3& m)
{
	double OneOverDeterminant = 1 / determinant(m);

	dmat3 Inverse;
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
