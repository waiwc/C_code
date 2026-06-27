// 
// Created By: Qilin.Ren 2018/05/29
// 

#include "glm/dmat3.hpp"
#include "glm/dmat4.hpp"
#include "glm/mat4.hpp"
#include "glm/dvec3.hpp"
#include "glm/dvec4.hpp"
#include "glm/dquat.hpp"
#include <limits>
#include <cmath>
#include <cstring>
#include <cassert>

CYBERTRON_BEGIN

CybertronCoreAPI dmat4::dmat4()
{
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			m[i][j] = 0;
		}
	}
}

CybertronCoreAPI dmat4::dmat4(const mat4& rhs)
{
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			m[i][j] = rhs.m[i][j];
		}
	}
}

CybertronCoreAPI dmat4::dmat4(const dmat3& rhs)
{
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			m[i][j] = rhs[i][j];
		}
	}

	m[0][3] = 0;
	m[1][3] = 0;
	m[2][3] = 0;

	m[3][0] = 0;
	m[3][1] = 0;
	m[3][2] = 0;
	m[3][3] = 1;
}

CybertronCoreAPI dmat4::dmat4(double s)
{
	m[0][0] = s;
	m[0][1] = 0;
	m[0][2] = 0;
	m[0][3] = 0;

	m[1][0] = 0;
	m[1][1] = s;
	m[1][2] = 0;
	m[1][3] = 0;

	m[2][0] = 0;
	m[2][1] = 0;
	m[2][2] = s;
	m[2][3] = 0;

	m[3][0] = 0;
	m[3][1] = 0;
	m[3][2] = 0;
	m[3][3] = s;
}

CybertronCoreAPI dmat4::dmat4(const dmat4& rhs)
{
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			m[i][j] = rhs.m[i][j];
		}
	}
}

CybertronCoreAPI dmat4::dmat4(
	const dvec4& val1,
	const dvec4& val2,
	const dvec4& val3,
	const dvec4& val4)
{
	memcpy(m[0], &val1, sizeof(dvec4));
	memcpy(m[1], &val2, sizeof(dvec4));
	memcpy(m[2], &val3, sizeof(dvec4));
	memcpy(m[3], &val4, sizeof(dvec4));
}

CybertronCoreAPI dmat4::dmat4(
	double x0, double y0, double z0, double w0,
	double x1, double y1, double z1, double w1,
	double x2, double y2, double z2, double w2,
	double x3, double y3, double z3, double w3)
{
	(*this)[0] = dvec4(x0, y0, z0, w0);
	(*this)[1] = dvec4(x1, y1, z1, w1);
	(*this)[2] = dvec4(x2, y2, z2, w2);
	(*this)[3] = dvec4(x3, y3, z3, w3);
}

CybertronCoreAPI bool dmat4::operator == (const dmat4& rhs) const
{
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			if (m[i][j] != rhs.m[i][j])
			{
				return false;
			}
		}
	}

	return true;
}

CybertronCoreAPI dvec4& dmat4::operator[](int col)
{
	double* pAddr = m[col];
	return *((dvec4*)pAddr);
}

CybertronCoreAPI const dvec4& dmat4::operator[](int col) const
{
	const double* pAddr = m[col];
	return *((dvec4*)pAddr);
}

CybertronCoreAPI dmat4 dmat4::operator * (const dmat4& m2) const
{
	const dmat4& m1 = *this;

	dvec4 const SrcA0 = m1[0];
	dvec4 const SrcA1 = m1[1];
	dvec4 const SrcA2 = m1[2];
	dvec4 const SrcA3 = m1[3];

	dvec4 const SrcB0 = m2[0];
	dvec4 const SrcB1 = m2[1];
	dvec4 const SrcB2 = m2[2];
	dvec4 const SrcB3 = m2[3];

	dmat4 Result;
	Result[0] = SrcA0 * SrcB0[0] + SrcA1 * SrcB0[1] + SrcA2 * SrcB0[2] + SrcA3 * SrcB0[3];
	Result[1] = SrcA0 * SrcB1[0] + SrcA1 * SrcB1[1] + SrcA2 * SrcB1[2] + SrcA3 * SrcB1[3];
	Result[2] = SrcA0 * SrcB2[0] + SrcA1 * SrcB2[1] + SrcA2 * SrcB2[2] + SrcA3 * SrcB2[3];
	Result[3] = SrcA0 * SrcB3[0] + SrcA1 * SrcB3[1] + SrcA2 * SrcB3[2] + SrcA3 * SrcB3[3];
	return Result;
}

CybertronCoreAPI dvec4 dmat4::operator * (const dvec4& v) const
{
	const dmat4& mm = *this;
	dvec4 const Mov0(v[0]);
	dvec4 const Mov1(v[1]);
	dvec4 const Mul0 = mm[0] * Mov0;
	dvec4 const Mul1 = mm[1] * Mov1;
	dvec4 const Add0 = Mul0 + Mul1;
	dvec4 const Mov2(v[2]);
	dvec4 const Mov3(v[3]);
	dvec4 const Mul2 = mm[2] * Mov2;
	dvec4 const Mul3 = mm[3] * Mov3;
	dvec4 const Add1 = Mul2 + Mul3;
	dvec4 const Add2 = Add0 + Add1;
	return Add2;

	/*
	return dvec4(
	m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2] + m[3][0] * v[3],
	m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2] + m[3][1] * v[3],
	m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2] + m[3][2] * v[3],
	m[0][3] * v[0] + m[1][3] * v[1] + m[2][3] * v[2] + m[3][3] * v[3]);
	*/
}

CybertronCoreAPI dvec3 dmat4::operator * (const dvec3& v) const
{
	dvec4 colVec(v, 1);
	return (this->operator * (colVec)).xyz() / colVec.w;
}

CybertronCoreAPI dmat4 dmat4::operator * (double val) const
{
	dmat4 result;
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			result.m[i][j] = m[i][j] * val;
		}
	}

	return result;
}

CybertronCoreAPI dmat4 dmat4::operator + (const dmat4& rhs) const
{
	dmat4 result;
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			result.m[i][j] = m[i][j] + rhs.m[i][j];
		}
	}

	return result;
}

CybertronCoreAPI dmat4 dmat4::operator - (const dmat4& rhs) const
{
	dmat4 result;
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			result.m[i][j] = m[i][j] - rhs.m[i][j];
		}
	}

	return result;
}

CybertronCoreAPI dmat4 dmat4::operator -() const
{
	dmat4 result;
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			result.m[i][j] = -m[i][j];
		}
	}

	return result;
}

CybertronCoreAPI dvec4 operator * (const dvec4& v, const dmat4& m)
{
	return dvec4(
		m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2] + m[0][3] * v[3],
		m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2] + m[1][3] * v[3],
		m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2] + m[2][3] * v[3],
		m[3][0] * v[0] + m[3][1] * v[1] + m[3][2] * v[2] + m[3][3] * v[3]);
}

CybertronCoreAPI dmat4 translate(const dvec3& v)
{
	dmat4 Result(1);
	Result[3] = Result[0] * v[0] + Result[1] * v[1] + Result[2] * v[2] + Result[3];
	return Result;
}

CybertronCoreAPI dmat4 scale(const dvec3& scale)
{
	dmat4 Result(1);
	Result[0] = Result[0] * scale[0];
	Result[1] = Result[1] * scale[1];
	Result[2] = Result[2] * scale[2];
	// Result[3] = Result[3];
	return Result;
}

CybertronCoreAPI dmat4 transpose(const dmat4& m)
{
	dmat4 result;

	result[0][0] = m[0][0];
	result[0][1] = m[1][0];
	result[0][2] = m[2][0];
	result[0][3] = m[3][0];

	result[1][0] = m[0][1];
	result[1][1] = m[1][1];
	result[1][2] = m[2][1];
	result[1][3] = m[3][1];

	result[2][0] = m[0][2];
	result[2][1] = m[1][2];
	result[2][2] = m[2][2];
	result[2][3] = m[3][2];

	result[3][0] = m[0][3];
	result[3][1] = m[1][3];
	result[3][2] = m[2][3];
	result[3][3] = m[3][3];
	return result;
}

CybertronCoreAPI dmat4 perspective(
	double fovy,
	double aspect,
	double zNear,
	double zFar)
{
	assert(std::abs(aspect - std::numeric_limits<double>::epsilon()) > static_cast<double>(0));

	double const tanHalfFovy = std::tan(fovy / static_cast<double>(2));

	dmat4 Result(static_cast<double>(0));
	Result[0][0] = static_cast<double>(1) / (aspect * tanHalfFovy);
	Result[1][1] = static_cast<double>(1) / (tanHalfFovy);
	Result[2][2] = -(zFar + zNear) / (zFar - zNear);
	Result[2][3] = -static_cast<double>(1);
	Result[3][2] = -(static_cast<double>(2) * zFar * zNear) / (zFar - zNear);
	return Result;
}

CybertronCoreAPI dmat4 ortho(
	double left,
	double right,
	double bottom,
	double top,
	double zNear,
	double zFar)
{
	dmat4 Result(1);
	Result[0][0] = static_cast<double>(2) / (right - left);
	Result[1][1] = static_cast<double>(2) / (top - bottom);
	Result[2][2] = -static_cast<double>(2) / (zFar - zNear);
	Result[3][0] = -(right + left) / (right - left);
	Result[3][1] = -(top + bottom) / (top - bottom);
	Result[3][2] = -(zFar + zNear) / (zFar - zNear);
	return Result;
}

CybertronCoreAPI dmat4 inverse(const dmat4& m)
{
	double Coef00 = m[2][2] * m[3][3] - m[3][2] * m[2][3];
	double Coef02 = m[1][2] * m[3][3] - m[3][2] * m[1][3];
	double Coef03 = m[1][2] * m[2][3] - m[2][2] * m[1][3];

	double Coef04 = m[2][1] * m[3][3] - m[3][1] * m[2][3];
	double Coef06 = m[1][1] * m[3][3] - m[3][1] * m[1][3];
	double Coef07 = m[1][1] * m[2][3] - m[2][1] * m[1][3];

	double Coef08 = m[2][1] * m[3][2] - m[3][1] * m[2][2];
	double Coef10 = m[1][1] * m[3][2] - m[3][1] * m[1][2];
	double Coef11 = m[1][1] * m[2][2] - m[2][1] * m[1][2];

	double Coef12 = m[2][0] * m[3][3] - m[3][0] * m[2][3];
	double Coef14 = m[1][0] * m[3][3] - m[3][0] * m[1][3];
	double Coef15 = m[1][0] * m[2][3] - m[2][0] * m[1][3];

	double Coef16 = m[2][0] * m[3][2] - m[3][0] * m[2][2];
	double Coef18 = m[1][0] * m[3][2] - m[3][0] * m[1][2];
	double Coef19 = m[1][0] * m[2][2] - m[2][0] * m[1][2];

	double Coef20 = m[2][0] * m[3][1] - m[3][0] * m[2][1];
	double Coef22 = m[1][0] * m[3][1] - m[3][0] * m[1][1];
	double Coef23 = m[1][0] * m[2][1] - m[2][0] * m[1][1];

	dvec4 Fac0(Coef00, Coef00, Coef02, Coef03);
	dvec4 Fac1(Coef04, Coef04, Coef06, Coef07);
	dvec4 Fac2(Coef08, Coef08, Coef10, Coef11);
	dvec4 Fac3(Coef12, Coef12, Coef14, Coef15);
	dvec4 Fac4(Coef16, Coef16, Coef18, Coef19);
	dvec4 Fac5(Coef20, Coef20, Coef22, Coef23);

	dvec4 Vec0(m[1][0], m[0][0], m[0][0], m[0][0]);
	dvec4 Vec1(m[1][1], m[0][1], m[0][1], m[0][1]);
	dvec4 Vec2(m[1][2], m[0][2], m[0][2], m[0][2]);
	dvec4 Vec3(m[1][3], m[0][3], m[0][3], m[0][3]);

	dvec4 Inv0(Vec1 * Fac0 - Vec2 * Fac1 + Vec3 * Fac2);
	dvec4 Inv1(Vec0 * Fac0 - Vec2 * Fac3 + Vec3 * Fac4);
	dvec4 Inv2(Vec0 * Fac1 - Vec1 * Fac3 + Vec3 * Fac5);
	dvec4 Inv3(Vec0 * Fac2 - Vec1 * Fac4 + Vec2 * Fac5);

	dvec4 SignA(+1, -1, +1, -1);
	dvec4 SignB(-1, +1, -1, +1);
	dmat4 Inverse(Inv0 * SignA, Inv1 * SignB, Inv2 * SignA, Inv3 * SignB);

	dvec4 Row0(Inverse[0][0], Inverse[1][0], Inverse[2][0], Inverse[3][0]);

	dvec4 Dot0(m[0] * Row0);
	double Dot1 = (Dot0.x + Dot0.y) + (Dot0.z + Dot0.w);

	double OneOverDeterminant = static_cast<double>(1) / Dot1;

	return Inverse * OneOverDeterminant;
}

CybertronCoreAPI dmat4 mix(const dmat4& val1, const dmat4& val2, double a)
{
	return val1 * (1 - a) + val2;
}

CybertronCoreAPI dmat4 lookAt(const dvec3& eye, const dvec3& center, const dvec3& up)
{
	dvec3 const f(normalize(center - eye));
	dvec3 const s(normalize(cross(f, up)));
	dvec3 const u(cross(s, f));

	dmat4 Result(1);
	Result[0][0] = s.x;
	Result[1][0] = s.y;
	Result[2][0] = s.z;
	Result[0][1] = u.x;
	Result[1][1] = u.y;
	Result[2][1] = u.z;
	Result[0][2] = -f.x;
	Result[1][2] = -f.y;
	Result[2][2] = -f.z;
	Result[3][0] = -dot(s, eye);
	Result[3][1] = -dot(u, eye);
	Result[3][2] = dot(f, eye);
	return Result;
}

CybertronCoreAPI double determinant(const dmat4& m)
{
	double SubFactor00 = m[2][2] * m[3][3] - m[3][2] * m[2][3];
	double SubFactor01 = m[2][1] * m[3][3] - m[3][1] * m[2][3];
	double SubFactor02 = m[2][1] * m[3][2] - m[3][1] * m[2][2];
	double SubFactor03 = m[2][0] * m[3][3] - m[3][0] * m[2][3];
	double SubFactor04 = m[2][0] * m[3][2] - m[3][0] * m[2][2];
	double SubFactor05 = m[2][0] * m[3][1] - m[3][0] * m[2][1];

	dvec4 DetCof(
		+(m[1][1] * SubFactor00 - m[1][2] * SubFactor01 + m[1][3] * SubFactor02),
		-(m[1][0] * SubFactor00 - m[1][2] * SubFactor03 + m[1][3] * SubFactor04),
		+(m[1][0] * SubFactor01 - m[1][1] * SubFactor03 + m[1][3] * SubFactor05),
		-(m[1][0] * SubFactor02 - m[1][1] * SubFactor04 + m[1][2] * SubFactor05));

	return
		m[0][0] * DetCof[0] + m[0][1] * DetCof[1] +
		m[0][2] * DetCof[2] + m[0][3] * DetCof[3];
}

static dvec3 __scale(dvec3 const& v, double desiredLength)
{
	return v * desiredLength / length(v);
}

static dvec3 __combine(
	dvec3 const & a,
	dvec3 const & b,
	double ascl, double bscl)
{
	return (a * ascl) + (b * bscl);
}

CybertronCoreAPI bool decompose(
	const dmat4& ModelMatrix,
	dvec3& Scale,
	dquat& Orientation,
	dvec3& Translation,
	dvec3& Skew,
	dvec4& Perspective)
{
	dmat4 LocalMatrix(ModelMatrix);

	// Normalize the matrix.
	if (LocalMatrix[3][3] == static_cast<double>(0))
		return false;

	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			LocalMatrix[i][j] /= LocalMatrix[3][3];

	// perspectiveMatrix is used to solve for perspective, but it also provides
	// an easy way to test for singularity of the upper 3x3 component.
	dmat4 PerspectiveMatrix(LocalMatrix);

	for (int i = 0; i < 3; i++)
		PerspectiveMatrix[i][3] = static_cast<double>(0);
	PerspectiveMatrix[3][3] = static_cast<double>(1);

	/// TODO: Fixme!
	if (determinant(PerspectiveMatrix) == static_cast<double>(0))
		return false;

	// First, isolate perspective.  This is the messiest.
	if (LocalMatrix[0][3] != static_cast<double>(0) || LocalMatrix[1][3] != static_cast<double>(0) || LocalMatrix[2][3] != static_cast<double>(0))
	{
		// rightHandSide is the right hand side of the equation.
		dvec4 RightHandSide;
		RightHandSide[0] = LocalMatrix[0][3];
		RightHandSide[1] = LocalMatrix[1][3];
		RightHandSide[2] = LocalMatrix[2][3];
		RightHandSide[3] = LocalMatrix[3][3];

		// Solve the equation by inverting PerspectiveMatrix and multiplying
		// rightHandSide by the inverse.  (This is the easiest way, not
		// necessarily the best.)
		dmat4 InversePerspectiveMatrix = inverse(PerspectiveMatrix);//   inverse(PerspectiveMatrix, inversePerspectiveMatrix);
		dmat4 TransposedInversePerspectiveMatrix = transpose(InversePerspectiveMatrix);//   transposeMatrix4(inversePerspectiveMatrix, transposedInversePerspectiveMatrix);

		Perspective = TransposedInversePerspectiveMatrix * RightHandSide;
		//  v4MulPointByMatrix(rightHandSide, transposedInversePerspectiveMatrix, perspectivePoint);

		// Clear the perspective partition
		LocalMatrix[0][3] = LocalMatrix[1][3] = LocalMatrix[2][3] = static_cast<double>(0);
		LocalMatrix[3][3] = static_cast<double>(1);
	}
	else
	{
		// No perspective.
		Perspective = dvec4(0, 0, 0, 1);
	}

	// Next take care of translation (easy).
	Translation = dvec3(LocalMatrix[3]);
	LocalMatrix[3] = dvec4(0, 0, 0, LocalMatrix[3].w);

	dvec3 Row[3], Pdum3;

	// Now get scale and shear.
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			Row[i][j] = LocalMatrix[i][j];

	// Compute X scale factor and normalize first row.
	Scale.x = length(Row[0]);// v3Length(Row[0]);

	Row[0] = __scale(Row[0], static_cast<double>(1));

	// Compute XY shear factor and make 2nd row orthogonal to 1st.
	Skew.z = dot(Row[0], Row[1]);
	Row[1] = __combine(Row[1], Row[0], static_cast<double>(1), -Skew.z);

	// Now, compute Y scale and normalize 2nd row.
	Scale.y = length(Row[1]);
	Row[1] = __scale(Row[1], static_cast<double>(1));
	Skew.z /= Scale.y;

	// Compute XZ and YZ shears, orthogonalize 3rd row.
	Skew.y = dot(Row[0], Row[2]);
	Row[2] = __combine(Row[2], Row[0], static_cast<double>(1), -Skew.y);
	Skew.x = dot(Row[1], Row[2]);
	Row[2] = __combine(Row[2], Row[1], static_cast<double>(1), -Skew.x);

	// Next, get Z scale and normalize 3rd row.
	Scale.z = length(Row[2]);
	Row[2] = __scale(Row[2], static_cast<double>(1));
	Skew.y /= Scale.z;
	Skew.x /= Scale.z;

	// At this point, the matrix (in rows[]) is orthonormal.
	// Check for a coordinate system flip.  If the determinant
	// is -1, then negate the matrix and the scaling factors.
	Pdum3 = cross(Row[1], Row[2]); // v3Cross(row[1], row[2], Pdum3);
	if (dot(Row[0], Pdum3) < 0)
	{
		for (int i = 0; i < 3; i++)
		{
			Scale[i] *= static_cast<double>(-1);
			Row[i] *= static_cast<double>(-1);
		}
	}

	// Now, get the rotations out, as described in the gem.

	// FIXME - Add the ability to return either quaternions (which are
	// easier to recompose with) or Euler angles (rx, ry, rz), which
	// are easier for authors to deal with. The latter will only be useful
	// when we fix https://bugs.webkit.org/show_bug.cgi?id=23799, so I
	// will leave the Euler angle code here for now.

	// ret.rotateY = asin(-Row[0][2]);
	// if (cos(ret.rotateY) != 0) {
	//     ret.rotateX = atan2(Row[1][2], Row[2][2]);
	//     ret.rotateZ = atan2(Row[0][1], Row[0][0]);
	// } else {
	//     ret.rotateX = atan2(-Row[2][0], Row[1][1]);
	//     ret.rotateZ = 0;
	// }

	int i, j, k = 0;
	double root, trace = Row[0].x + Row[1].y + Row[2].z;
	if (trace > static_cast<double>(0))
	{
		root = std::sqrt(trace + static_cast<double>(1.0));
		Orientation.w = static_cast<double>(0.5) * root;
		root = static_cast<double>(0.5) / root;
		Orientation.x = root * (Row[1].z - Row[2].y);
		Orientation.y = root * (Row[2].x - Row[0].z);
		Orientation.z = root * (Row[0].y - Row[1].x);
	} // End if > 0
	else
	{
		static int Next[3] = { 1, 2, 0 };
		i = 0;
		if (Row[1].y > Row[0].x) i = 1;
		if (Row[2].z > Row[i][i]) i = 2;
		j = Next[i];
		k = Next[j];

		root = std::sqrt(Row[i][i] - Row[j][j] - Row[k][k] + static_cast<double>(1.0));

		Orientation[i] = static_cast<double>(0.5) * root;
		root = static_cast<double>(0.5) / root;
		Orientation[j] = root * (Row[i][j] + Row[j][i]);
		Orientation[k] = root * (Row[i][k] + Row[k][i]);
		Orientation.w = root * (Row[j][k] - Row[k][j]);
	} // End if <= 0

	return true;
}

CybertronCoreAPI dmat4 rotate(double angle, const dvec3& v)
{
	double const a = angle;
	double const c = std::cos(a);
	double const s = std::sin(a);

	dvec3 axis(normalize(v));
	dvec3 temp((double(1) - c) * axis);

	dmat4 Rotate;
	Rotate[0][0] = c + temp[0] * axis[0];
	Rotate[0][1] = 0 + temp[0] * axis[1] + s * axis[2];
	Rotate[0][2] = 0 + temp[0] * axis[2] - s * axis[1];

	Rotate[1][0] = 0 + temp[1] * axis[0] - s * axis[2];
	Rotate[1][1] = c + temp[1] * axis[1];
	Rotate[1][2] = 0 + temp[1] * axis[2] + s * axis[0];

	Rotate[2][0] = 0 + temp[2] * axis[0] + s * axis[1];
	Rotate[2][1] = 0 + temp[2] * axis[1] - s * axis[0];
	Rotate[2][2] = c + temp[2] * axis[2];

	dmat4 m(1);
	dmat4 Result;
	Result[0] = m[0] * Rotate[0][0] + m[1] * Rotate[0][1] + m[2] * Rotate[0][2];
	Result[1] = m[0] * Rotate[1][0] + m[1] * Rotate[1][1] + m[2] * Rotate[1][2];
	Result[2] = m[0] * Rotate[2][0] + m[1] * Rotate[2][1] + m[2] * Rotate[2][2];
	Result[3] = m[3];
	return Result;
}

CybertronCoreAPI dmat4 matrixCompMult(const dmat4& lhs, const dmat4& rhs)
{
	dmat4 result;
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			result[i][j] = lhs[i][j] * rhs[i][j];
		}
	}

	return result;
}

CYBERTRON_END
