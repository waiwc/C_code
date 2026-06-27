// 
// Created By: Qilin.Ren 2018/05/29
// 

#include "glm/mat3.hpp"
#include "glm/mat4.hpp"
#include "glm/vec3.hpp"
#include "glm/vec4.hpp"
#include "glm/quat.hpp"
#include <limits>
#include <cmath>
#include <cstring>
#include <cassert>

CYBERTRON_BEGIN

CybertronCoreAPI mat4::mat4()
{
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			m[i][j] = 0;
		}
	}
}

CybertronCoreAPI mat4::mat4(const mat3& rhs)
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

CybertronCoreAPI mat4::mat4(float s)
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

CybertronCoreAPI mat4::mat4(const mat4& rhs)
{
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			m[i][j] = rhs.m[i][j];
		}
	}
}

CybertronCoreAPI mat4::mat4(
	const vec4& val1,
	const vec4& val2,
	const vec4& val3,
	const vec4& val4)
{
	memcpy(m[0], &val1, sizeof(vec4));
	memcpy(m[1], &val2, sizeof(vec4));
	memcpy(m[2], &val3, sizeof(vec4));
	memcpy(m[3], &val4, sizeof(vec4));
}

CybertronCoreAPI mat4::mat4(
	float x0, float y0, float z0, float w0,
	float x1, float y1, float z1, float w1,
	float x2, float y2, float z2, float w2,
	float x3, float y3, float z3, float w3)
{
	(*this)[0] = vec4(x0, y0, z0, w0);
	(*this)[1] = vec4(x1, y1, z1, w1);
	(*this)[2] = vec4(x2, y2, z2, w2);
	(*this)[3] = vec4(x3, y3, z3, w3);
}

CybertronCoreAPI bool mat4::operator == (const mat4& rhs) const
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

CybertronCoreAPI vec4& mat4::operator[](int col)
{
	float* pAddr = m[col];
	return *((vec4*)pAddr);
}

CybertronCoreAPI const vec4& mat4::operator[](int col) const
{
	const float* pAddr = m[col];
	return *((vec4*)pAddr);
}

CybertronCoreAPI mat4 mat4::operator * (const mat4& m2) const
{
	const mat4& m1 = *this;

	vec4 const SrcA0 = m1[0];
	vec4 const SrcA1 = m1[1];
	vec4 const SrcA2 = m1[2];
	vec4 const SrcA3 = m1[3];

	vec4 const SrcB0 = m2[0];
	vec4 const SrcB1 = m2[1];
	vec4 const SrcB2 = m2[2];
	vec4 const SrcB3 = m2[3];

	mat4 Result;
	Result[0] = SrcA0 * SrcB0[0] + SrcA1 * SrcB0[1] + SrcA2 * SrcB0[2] + SrcA3 * SrcB0[3];
	Result[1] = SrcA0 * SrcB1[0] + SrcA1 * SrcB1[1] + SrcA2 * SrcB1[2] + SrcA3 * SrcB1[3];
	Result[2] = SrcA0 * SrcB2[0] + SrcA1 * SrcB2[1] + SrcA2 * SrcB2[2] + SrcA3 * SrcB2[3];
	Result[3] = SrcA0 * SrcB3[0] + SrcA1 * SrcB3[1] + SrcA2 * SrcB3[2] + SrcA3 * SrcB3[3];
	return Result;
}

CybertronCoreAPI vec4 mat4::operator * (const vec4& v) const
{
	const mat4& mm = *this;
	vec4 const Mov0(v[0]);
	vec4 const Mov1(v[1]);
	vec4 const Mul0 = mm[0] * Mov0;
	vec4 const Mul1 = mm[1] * Mov1;
	vec4 const Add0 = Mul0 + Mul1;
	vec4 const Mov2(v[2]);
	vec4 const Mov3(v[3]);
	vec4 const Mul2 = mm[2] * Mov2;
	vec4 const Mul3 = mm[3] * Mov3;
	vec4 const Add1 = Mul2 + Mul3;
	vec4 const Add2 = Add0 + Add1;
	return Add2;

	/*
	return vec4(
	m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2] + m[3][0] * v[3],
	m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2] + m[3][1] * v[3],
	m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2] + m[3][2] * v[3],
	m[0][3] * v[0] + m[1][3] * v[1] + m[2][3] * v[2] + m[3][3] * v[3]);
	*/
}

CybertronCoreAPI vec3 mat4::operator * (const vec3& v) const
{
	vec4 colVec(v, 1);
	return (this->operator * (colVec)).xyz() / colVec.w;
}

CybertronCoreAPI mat4 mat4::operator * (float val) const
{
	mat4 result;
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			result.m[i][j] = m[i][j] * val;
		}
	}

	return result;
}

CybertronCoreAPI mat4 mat4::operator + (const mat4& rhs) const
{
	mat4 result;
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			result.m[i][j] = m[i][j] + rhs.m[i][j];
		}
	}

	return result;
}

CybertronCoreAPI mat4 mat4::operator - (const mat4& rhs) const
{
	mat4 result;
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			result.m[i][j] = m[i][j] - rhs.m[i][j];
		}
	}

	return result;
}

CybertronCoreAPI mat4 mat4::operator -() const
{
	mat4 result;
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			result.m[i][j] = -m[i][j];
		}
	}

	return result;
}

CybertronCoreAPI vec4 operator * (const vec4& v, const mat4& m)
{
	return vec4(
		m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2] + m[0][3] * v[3],
		m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2] + m[1][3] * v[3],
		m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2] + m[2][3] * v[3],
		m[3][0] * v[0] + m[3][1] * v[1] + m[3][2] * v[2] + m[3][3] * v[3]);
}

CybertronCoreAPI mat4 translate(const vec3& v)
{
	mat4 Result(1);
	Result[3] = Result[0] * v[0] + Result[1] * v[1] + Result[2] * v[2] + Result[3];
	return Result;
}

CybertronCoreAPI mat4 scale(const vec3& scale)
{
	mat4 Result(1);
	Result[0] = Result[0] * scale[0];
	Result[1] = Result[1] * scale[1];
	Result[2] = Result[2] * scale[2];
	// Result[3] = Result[3];
	return Result;
}

CybertronCoreAPI mat4 transpose(const mat4& m)
{
	mat4 result;

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

CybertronCoreAPI mat4 perspective(
	float fovy,
	float aspect,
	float zNear,
	float zFar)
{
	assert(fabs(aspect - std::numeric_limits<float>::epsilon()) > static_cast<float>(0));

	float const tanHalfFovy = tanf(fovy / static_cast<float>(2));

	mat4 Result(static_cast<float>(0));
	Result[0][0] = static_cast<float>(1) / (aspect * tanHalfFovy);
	Result[1][1] = static_cast<float>(1) / (tanHalfFovy);
	Result[2][2] = -(zFar + zNear) / (zFar - zNear);
	Result[2][3] = -static_cast<float>(1);
	Result[3][2] = -(static_cast<float>(2) * zFar * zNear) / (zFar - zNear);
	return Result;
}

CybertronCoreAPI mat4 ortho(
	float left,
	float right,
	float bottom,
	float top,
	float zNear,
	float zFar)
{
	mat4 Result(1);
	Result[0][0] = static_cast<float>(2) / (right - left);
	Result[1][1] = static_cast<float>(2) / (top - bottom);
	Result[2][2] = -static_cast<float>(2) / (zFar - zNear);
	Result[3][0] = -(right + left) / (right - left);
	Result[3][1] = -(top + bottom) / (top - bottom);
	Result[3][2] = -(zFar + zNear) / (zFar - zNear);
	return Result;
}

CybertronCoreAPI mat4 inverse(const mat4& m)
{
	float Coef00 = m[2][2] * m[3][3] - m[3][2] * m[2][3];
	float Coef02 = m[1][2] * m[3][3] - m[3][2] * m[1][3];
	float Coef03 = m[1][2] * m[2][3] - m[2][2] * m[1][3];

	float Coef04 = m[2][1] * m[3][3] - m[3][1] * m[2][3];
	float Coef06 = m[1][1] * m[3][3] - m[3][1] * m[1][3];
	float Coef07 = m[1][1] * m[2][3] - m[2][1] * m[1][3];

	float Coef08 = m[2][1] * m[3][2] - m[3][1] * m[2][2];
	float Coef10 = m[1][1] * m[3][2] - m[3][1] * m[1][2];
	float Coef11 = m[1][1] * m[2][2] - m[2][1] * m[1][2];

	float Coef12 = m[2][0] * m[3][3] - m[3][0] * m[2][3];
	float Coef14 = m[1][0] * m[3][3] - m[3][0] * m[1][3];
	float Coef15 = m[1][0] * m[2][3] - m[2][0] * m[1][3];

	float Coef16 = m[2][0] * m[3][2] - m[3][0] * m[2][2];
	float Coef18 = m[1][0] * m[3][2] - m[3][0] * m[1][2];
	float Coef19 = m[1][0] * m[2][2] - m[2][0] * m[1][2];

	float Coef20 = m[2][0] * m[3][1] - m[3][0] * m[2][1];
	float Coef22 = m[1][0] * m[3][1] - m[3][0] * m[1][1];
	float Coef23 = m[1][0] * m[2][1] - m[2][0] * m[1][1];

	vec4 Fac0(Coef00, Coef00, Coef02, Coef03);
	vec4 Fac1(Coef04, Coef04, Coef06, Coef07);
	vec4 Fac2(Coef08, Coef08, Coef10, Coef11);
	vec4 Fac3(Coef12, Coef12, Coef14, Coef15);
	vec4 Fac4(Coef16, Coef16, Coef18, Coef19);
	vec4 Fac5(Coef20, Coef20, Coef22, Coef23);

	vec4 Vec0(m[1][0], m[0][0], m[0][0], m[0][0]);
	vec4 Vec1(m[1][1], m[0][1], m[0][1], m[0][1]);
	vec4 Vec2(m[1][2], m[0][2], m[0][2], m[0][2]);
	vec4 Vec3(m[1][3], m[0][3], m[0][3], m[0][3]);

	vec4 Inv0(Vec1 * Fac0 - Vec2 * Fac1 + Vec3 * Fac2);
	vec4 Inv1(Vec0 * Fac0 - Vec2 * Fac3 + Vec3 * Fac4);
	vec4 Inv2(Vec0 * Fac1 - Vec1 * Fac3 + Vec3 * Fac5);
	vec4 Inv3(Vec0 * Fac2 - Vec1 * Fac4 + Vec2 * Fac5);

	vec4 SignA(+1, -1, +1, -1);
	vec4 SignB(-1, +1, -1, +1);
	mat4 Inverse(Inv0 * SignA, Inv1 * SignB, Inv2 * SignA, Inv3 * SignB);

	vec4 Row0(Inverse[0][0], Inverse[1][0], Inverse[2][0], Inverse[3][0]);

	vec4 Dot0(m[0] * Row0);
	float Dot1 = (Dot0.x + Dot0.y) + (Dot0.z + Dot0.w);

	float OneOverDeterminant = static_cast<float>(1) / Dot1;

	return Inverse * OneOverDeterminant;
}

CybertronCoreAPI mat4 mix(const mat4& val1, const mat4& val2, float a)
{
	return val1 * (1 - a) + val2;
}

CybertronCoreAPI mat4 lookAt(const vec3& eye, const vec3& center, const vec3& up)
{
	vec3 const f(normalize(center - eye));
	vec3 const s(normalize(cross(f, up)));
	vec3 const u(cross(s, f));

	mat4 Result(1);
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

CybertronCoreAPI float determinant(const mat4& m)
{
	float SubFactor00 = m[2][2] * m[3][3] - m[3][2] * m[2][3];
	float SubFactor01 = m[2][1] * m[3][3] - m[3][1] * m[2][3];
	float SubFactor02 = m[2][1] * m[3][2] - m[3][1] * m[2][2];
	float SubFactor03 = m[2][0] * m[3][3] - m[3][0] * m[2][3];
	float SubFactor04 = m[2][0] * m[3][2] - m[3][0] * m[2][2];
	float SubFactor05 = m[2][0] * m[3][1] - m[3][0] * m[2][1];

	vec4 DetCof(
		+(m[1][1] * SubFactor00 - m[1][2] * SubFactor01 + m[1][3] * SubFactor02),
		-(m[1][0] * SubFactor00 - m[1][2] * SubFactor03 + m[1][3] * SubFactor04),
		+(m[1][0] * SubFactor01 - m[1][1] * SubFactor03 + m[1][3] * SubFactor05),
		-(m[1][0] * SubFactor02 - m[1][1] * SubFactor04 + m[1][2] * SubFactor05));

	return
		m[0][0] * DetCof[0] + m[0][1] * DetCof[1] +
		m[0][2] * DetCof[2] + m[0][3] * DetCof[3];
}

static vec3 __scale(vec3 const& v, float desiredLength)
{
	return v * desiredLength / length(v);
}

static vec3 __combine(
	vec3 const & a,
	vec3 const & b,
	float ascl, float bscl)
{
	return (a * ascl) + (b * bscl);
}

CybertronCoreAPI bool decompose(
	const mat4& ModelMatrix,
	vec3& Scale,
	quat& Orientation,
	vec3& Translation,
	vec3& Skew,
	vec4& Perspective)
{
	mat4 LocalMatrix(ModelMatrix);

	// Normalize the matrix.
	if (LocalMatrix[3][3] == static_cast<float>(0))
		return false;

	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			LocalMatrix[i][j] /= LocalMatrix[3][3];

	// perspectiveMatrix is used to solve for perspective, but it also provides
	// an easy way to test for singularity of the upper 3x3 component.
	mat4 PerspectiveMatrix(LocalMatrix);

	for (int i = 0; i < 3; i++)
		PerspectiveMatrix[i][3] = static_cast<float>(0);
	PerspectiveMatrix[3][3] = static_cast<float>(1);

	/// TODO: Fixme!
	if (determinant(PerspectiveMatrix) == static_cast<float>(0))
		return false;

	// First, isolate perspective.  This is the messiest.
	if (LocalMatrix[0][3] != static_cast<float>(0) || LocalMatrix[1][3] != static_cast<float>(0) || LocalMatrix[2][3] != static_cast<float>(0))
	{
		// rightHandSide is the right hand side of the equation.
		vec4 RightHandSide;
		RightHandSide[0] = LocalMatrix[0][3];
		RightHandSide[1] = LocalMatrix[1][3];
		RightHandSide[2] = LocalMatrix[2][3];
		RightHandSide[3] = LocalMatrix[3][3];

		// Solve the equation by inverting PerspectiveMatrix and multiplying
		// rightHandSide by the inverse.  (This is the easiest way, not
		// necessarily the best.)
		mat4 InversePerspectiveMatrix = inverse(PerspectiveMatrix);//   inverse(PerspectiveMatrix, inversePerspectiveMatrix);
		mat4 TransposedInversePerspectiveMatrix = transpose(InversePerspectiveMatrix);//   transposeMatrix4(inversePerspectiveMatrix, transposedInversePerspectiveMatrix);

		Perspective = TransposedInversePerspectiveMatrix * RightHandSide;
		//  v4MulPointByMatrix(rightHandSide, transposedInversePerspectiveMatrix, perspectivePoint);

		// Clear the perspective partition
		LocalMatrix[0][3] = LocalMatrix[1][3] = LocalMatrix[2][3] = static_cast<float>(0);
		LocalMatrix[3][3] = static_cast<float>(1);
	}
	else
	{
		// No perspective.
		Perspective = vec4(0, 0, 0, 1);
	}

	// Next take care of translation (easy).
	Translation = vec3(LocalMatrix[3]);
	LocalMatrix[3] = vec4(0, 0, 0, LocalMatrix[3].w);

	vec3 Row[3], Pdum3;

	// Now get scale and shear.
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			Row[i][j] = LocalMatrix[i][j];

	// Compute X scale factor and normalize first row.
	Scale.x = length(Row[0]);// v3Length(Row[0]);

	Row[0] = __scale(Row[0], static_cast<float>(1));

	// Compute XY shear factor and make 2nd row orthogonal to 1st.
	Skew.z = dot(Row[0], Row[1]);
	Row[1] = __combine(Row[1], Row[0], static_cast<float>(1), -Skew.z);

	// Now, compute Y scale and normalize 2nd row.
	Scale.y = length(Row[1]);
	Row[1] = __scale(Row[1], static_cast<float>(1));
	Skew.z /= Scale.y;

	// Compute XZ and YZ shears, orthogonalize 3rd row.
	Skew.y = dot(Row[0], Row[2]);
	Row[2] = __combine(Row[2], Row[0], static_cast<float>(1), -Skew.y);
	Skew.x = dot(Row[1], Row[2]);
	Row[2] = __combine(Row[2], Row[1], static_cast<float>(1), -Skew.x);

	// Next, get Z scale and normalize 3rd row.
	Scale.z = length(Row[2]);
	Row[2] = __scale(Row[2], static_cast<float>(1));
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
			Scale[i] *= static_cast<float>(-1);
			Row[i] *= static_cast<float>(-1);
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
	float root, trace = Row[0].x + Row[1].y + Row[2].z;
	if (trace > static_cast<float>(0))
	{
		root = sqrt(trace + static_cast<float>(1.0));
		Orientation.w = static_cast<float>(0.5) * root;
		root = static_cast<float>(0.5) / root;
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

		root = sqrt(Row[i][i] - Row[j][j] - Row[k][k] + static_cast<float>(1.0));

		Orientation[i] = static_cast<float>(0.5) * root;
		root = static_cast<float>(0.5) / root;
		Orientation[j] = root * (Row[i][j] + Row[j][i]);
		Orientation[k] = root * (Row[i][k] + Row[k][i]);
		Orientation.w = root * (Row[j][k] - Row[k][j]);
	} // End if <= 0

	return true;
}

CybertronCoreAPI mat4 rotate(float angle, const vec3& v)
{
	float const a = angle;
	float const c = cos(a);
	float const s = sin(a);

	vec3 axis(normalize(v));
	vec3 temp((float(1) - c) * axis);

	mat4 Rotate;
	Rotate[0][0] = c + temp[0] * axis[0];
	Rotate[0][1] = 0 + temp[0] * axis[1] + s * axis[2];
	Rotate[0][2] = 0 + temp[0] * axis[2] - s * axis[1];

	Rotate[1][0] = 0 + temp[1] * axis[0] - s * axis[2];
	Rotate[1][1] = c + temp[1] * axis[1];
	Rotate[1][2] = 0 + temp[1] * axis[2] + s * axis[0];

	Rotate[2][0] = 0 + temp[2] * axis[0] + s * axis[1];
	Rotate[2][1] = 0 + temp[2] * axis[1] - s * axis[0];
	Rotate[2][2] = c + temp[2] * axis[2];

	mat4 m(1);
	mat4 Result;
	Result[0] = m[0] * Rotate[0][0] + m[1] * Rotate[0][1] + m[2] * Rotate[0][2];
	Result[1] = m[0] * Rotate[1][0] + m[1] * Rotate[1][1] + m[2] * Rotate[1][2];
	Result[2] = m[0] * Rotate[2][0] + m[1] * Rotate[2][1] + m[2] * Rotate[2][2];
	Result[3] = m[3];
	return Result;
}

CybertronCoreAPI mat4 matrixCompMult(const mat4& lhs, const mat4& rhs)
{
	mat4 result;
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			result[i][j] = lhs[i][j] * rhs[i][j];
		}
	}

	return result;
}

CybertronCoreAPI mat4 eulerAngleX(float angleX)
{
	float cosX = std::cos(angleX);
	float sinX = std::sin(angleX);

	return mat4(
		float(1), float(0), float(0), float(0),
		float(0), cosX, sinX, float(0),
		float(0), -sinX, cosX, float(0),
		float(0), float(0), float(0), float(1));
}

CybertronCoreAPI mat4 eulerAngleY(float angleY)
{
	float cosY = std::cos(angleY);
	float sinY = std::sin(angleY);

	return mat4(
		cosY, float(0), -sinY, float(0),
		float(0), float(1), float(0), float(0),
		sinY, float(0), cosY, float(0),
		float(0), float(0), float(0), float(1));
}

CybertronCoreAPI mat4 eulerAngleZ(float angleZ)
{
	float cosZ = std::cos(angleZ);
	float sinZ = std::sin(angleZ);

	return mat4(
		cosZ, sinZ, float(0), float(0),
		-sinZ, cosZ, float(0), float(0),
		float(0), float(0), float(1), float(0),
		float(0), float(0), float(0), float(1));
}

CYBERTRON_END
