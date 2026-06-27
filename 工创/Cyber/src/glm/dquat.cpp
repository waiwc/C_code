// 
// Created By: Qilin.Ren 2018/05/29
// 

#include "glm/dquat.hpp"
#include "glm/dmat4.hpp"
#include "glm/dmat3.hpp"
#include "glm/dvec3.hpp"
#include "glm/dvec4.hpp"
#include <assert.h>
#include <cmath>
#include <float.h>


CYBERTRON_BEGIN

static double mixf(double val1, double val2, double a)
{
	return val1 * (1 - a) + val2 * a;
}

CybertronCoreAPI dquat::dquat(const dmat3& m)
{
	*this = quat_cast(m);
}

CybertronCoreAPI dquat::dquat(const dmat4& m)
{
	*this = quat_cast(m);
}

CybertronCoreAPI dquat::dquat(const dvec3& u, const dvec3& v)
{
	dvec3 const LocalW(cross(u, v));
	double Dot = dot(u, v);
	dquat q(double(1) + Dot, LocalW.x, LocalW.y, LocalW.z);

	*this = normalize(q);
}

dvec3 cos(const dvec3& angles)
{
	return dvec3(
		std::cos(angles.x),
		std::cos(angles.y),
		std::cos(angles.z));
}

dvec3 sin(const dvec3& angles)
{
	return dvec3(
		std::sin(angles.x),
		std::sin(angles.y),
		std::sin(angles.z));
}

CybertronCoreAPI dquat::dquat(const dvec3& eulerAngle)
{
	dvec3 c = cos(eulerAngle * double(0.5));
	dvec3 s = sin(eulerAngle * double(0.5));

	this->w = c.x * c.y * c.z + s.x * s.y * s.z;
	this->x = s.x * c.y * c.z - c.x * s.y * s.z;
	this->y = c.x * s.y * c.z + s.x * c.y * s.z;
	this->z = c.x * c.y * s.z - s.x * s.y * c.z;
}

CybertronCoreAPI dquat dquat::operator * (const dquat& rhs) const
{
	dquat lhs(*this);
	lhs *= rhs;
	return lhs;
}

CybertronCoreAPI dquat dquat::operator * (double rhs) const
{
	return dquat(w * rhs, x * rhs, y * rhs, z * rhs);
}

CybertronCoreAPI dquat dquat::operator / (double rhs) const
{
	return dquat(w / rhs, x / rhs, y / rhs, z / rhs);
}

CybertronCoreAPI dquat dquat::operator + (const dquat& rhs) const
{
	return dquat(
		w + rhs.w,
		x + rhs.x,
		y + rhs.y,
		z + rhs.z);
}

CybertronCoreAPI dquat dquat::operator - (const dquat& rhs) const
{
	return dquat(
		w - rhs.w,
		x - rhs.x,
		y - rhs.y,
		z - rhs.z);
}

CybertronCoreAPI dquat dquat::operator - () const
{
	const dquat& q = *this;
	return dquat(-q.w, -q.x, -q.y, -q.z);
}

CybertronCoreAPI dquat& dquat::operator *= (const dquat& r)
{
	dquat const p(*this);
	dquat const q(r);

	this->w = p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z;
	this->x = p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y;
	this->y = p.w * q.y + p.y * q.w + p.z * q.x - p.x * q.z;
	this->z = p.w * q.z + p.z * q.w + p.x * q.y - p.y * q.x;
	return *this;
}

CybertronCoreAPI dvec3 dquat::operator * (const dvec3& v) const
{
	const dquat& q = *this;
	dvec3 const QuatVector(q.x, q.y, q.z);
	dvec3 const uv(cross(QuatVector, v));
	dvec3 const uuv(cross(QuatVector, uv));

	return v + ((uv * q.w) + uuv) * static_cast<double>(2);
}

CybertronCoreAPI dquat operator * (double lhs, const dquat& rhs)
{
	return dquat(lhs * rhs.w, lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
}

CybertronCoreAPI dquat quat_cast(const dmat3& m)
{
	double fourXSquaredMinus1 = m[0][0] - m[1][1] - m[2][2];
	double fourYSquaredMinus1 = m[1][1] - m[0][0] - m[2][2];
	double fourZSquaredMinus1 = m[2][2] - m[0][0] - m[1][1];
	double fourWSquaredMinus1 = m[0][0] + m[1][1] + m[2][2];

	int biggestIndex = 0;
	double fourBiggestSquaredMinus1 = fourWSquaredMinus1;
	if (fourXSquaredMinus1 > fourBiggestSquaredMinus1)
	{
		fourBiggestSquaredMinus1 = fourXSquaredMinus1;
		biggestIndex = 1;
	}
	if (fourYSquaredMinus1 > fourBiggestSquaredMinus1)
	{
		fourBiggestSquaredMinus1 = fourYSquaredMinus1;
		biggestIndex = 2;
	}
	if (fourZSquaredMinus1 > fourBiggestSquaredMinus1)
	{
		fourBiggestSquaredMinus1 = fourZSquaredMinus1;
		biggestIndex = 3;
	}

	double biggestVal = std::sqrt(fourBiggestSquaredMinus1 + double(1)) * double(0.5);
	double mult = static_cast<double>(0.25) / biggestVal;

	dquat Result;
	switch (biggestIndex)
	{
	case 0:
		Result.w = biggestVal;
		Result.x = (m[1][2] - m[2][1]) * mult;
		Result.y = (m[2][0] - m[0][2]) * mult;
		Result.z = (m[0][1] - m[1][0]) * mult;
		break;
	case 1:
		Result.w = (m[1][2] - m[2][1]) * mult;
		Result.x = biggestVal;
		Result.y = (m[0][1] + m[1][0]) * mult;
		Result.z = (m[2][0] + m[0][2]) * mult;
		break;
	case 2:
		Result.w = (m[2][0] - m[0][2]) * mult;
		Result.x = (m[0][1] + m[1][0]) * mult;
		Result.y = biggestVal;
		Result.z = (m[1][2] + m[2][1]) * mult;
		break;
	case 3:
		Result.w = (m[0][1] - m[1][0]) * mult;
		Result.x = (m[2][0] + m[0][2]) * mult;
		Result.y = (m[1][2] + m[2][1]) * mult;
		Result.z = biggestVal;
		break;

	default:
		// Silence a -Wswitch-default warning in GCC. Should never actually get here. Assert is just for sanity.
		assert(false);
		break;
	}
	return Result;
}

CybertronCoreAPI dquat quat_cast(const dmat4& m)
{
	return quat_cast(dmat3(m));
}

CybertronCoreAPI dmat3 mat3_cast(const dquat& q)
{
	dmat3 Result(double(1));
	double qxx(q.x * q.x);
	double qyy(q.y * q.y);
	double qzz(q.z * q.z);
	double qxz(q.x * q.z);
	double qxy(q.x * q.y);
	double qyz(q.y * q.z);
	double qwx(q.w * q.x);
	double qwy(q.w * q.y);
	double qwz(q.w * q.z);

	Result[0][0] = 1 - 2 * (qyy + qzz);
	Result[0][1] = 2 * (qxy + qwz);
	Result[0][2] = 2 * (qxz - qwy);

	Result[1][0] = 2 * (qxy - qwz);
	Result[1][1] = 1 - 2 * (qxx + qzz);
	Result[1][2] = 2 * (qyz + qwx);

	Result[2][0] = 2 * (qxz + qwy);
	Result[2][1] = 2 * (qyz - qwx);
	Result[2][2] = 1 - 2 * (qxx + qyy);
	return Result;
}

CybertronCoreAPI dmat4 mat4_cast(const dquat& q)
{
	return dmat4(mat3_cast(q));
}

CybertronCoreAPI double dot(const dquat& x, const dquat& y)
{
	dvec4 tmp(x.x * y.x, x.y * y.y, x.z * y.z, x.w * y.w);
	return (tmp.x + tmp.y) + (tmp.z + tmp.w);
}

CybertronCoreAPI double length(const dquat& q)
{
	return std::sqrt(dot(q, q));
}

CybertronCoreAPI dquat lerp(const dquat& x, const dquat& y, double a)
{
	// Lerp is only defined in [0, 1]
	assert(a >= static_cast<double>(0));
	assert(a <= static_cast<double>(1));

	return x * (double(1) - a) + (y * a);
}

CybertronCoreAPI dquat slerp(const dquat& x, const dquat& y, double a)
{
	dquat z = y;

	double cosTheta = dot(x, y);

	// If cosTheta < 0, the interpolation will take the long way around the sphere. 
	// To fix this, one dquat must be negated.
	if (cosTheta < double(0))
	{
		z = -y;
		cosTheta = -cosTheta;
	}

	// Perform a linear interpolation when cosTheta is close to 1 to avoid side effect of sin(angle) becoming a zero denominator
	if (cosTheta > double(1) - FLT_EPSILON)
	{
		// Linear interpolation
		return dquat(
			mixf(x.w, z.w, a),
			mixf(x.x, z.x, a),
			mixf(x.y, z.y, a),
			mixf(x.z, z.z, a));
	}
	else
	{
		// Essential Mathematics, page 467
		double angle = std::acos(cosTheta);
		return (std::sin((double(1) - a) * angle) * x + std::sin(a * angle) * z) / std::sin(angle);
	}
}

CybertronCoreAPI double roll(dquat const & q)
{
	return double(std::atan2(double(2) * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z));
}

CybertronCoreAPI double pitch(dquat const & q)
{
	return double(std::atan2(double(2) * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z));
}

inline double clamp(double x, double minVal, double maxVal)
{
	if (x < minVal)
	{
		x = minVal;
	}
	if (x > maxVal)
	{
		x = maxVal;
	}

	return x;
}

CybertronCoreAPI double yaw(dquat const & q)
{
	return std::asin(clamp(double(-2) * (q.x * q.z - q.w * q.y), -1.0, 1.0));
}

CybertronCoreAPI dvec3 eulerAngles(const dquat& x)
{
	// NOTE: Definition of pitch/yaw/roll coule be different based on scenario.
	//       Just take the three parameters as rotation of X, Y and Z axis.
	return dvec3(pitch(x), yaw(x), roll(x));
}

CybertronCoreAPI dmat4 eulerAngleXYZ(double t1, double t2, double t3)
{
	double c1 = std::cos(-t1);
	double c2 = std::cos(-t2);
	double c3 = std::cos(-t3);
	double s1 = std::sin(-t1);
	double s2 = std::sin(-t2);
	double s3 = std::sin(-t3);

	dmat4 Result;
	Result[0][0] = c2 * c3;
	Result[0][1] = -c1 * s3 + s1 * s2 * c3;
	Result[0][2] = s1 * s3 + c1 * s2 * c3;
	Result[0][3] = static_cast<double>(0);
	Result[1][0] = c2 * s3;
	Result[1][1] = c1 * c3 + s1 * s2 * s3;
	Result[1][2] = -s1 * c3 + c1 * s2 * s3;
	Result[1][3] = static_cast<double>(0);
	Result[2][0] = -s2;
	Result[2][1] = s1 * c2;
	Result[2][2] = c1 * c2;
	Result[2][3] = static_cast<double>(0);
	Result[3][0] = static_cast<double>(0);
	Result[3][1] = static_cast<double>(0);
	Result[3][2] = static_cast<double>(0);
	Result[3][3] = static_cast<double>(1);
	return Result;
}

CybertronCoreAPI dquat normalize(const dquat& q)
{
	double len = length(q);
	if (len <= double(0)) // Problem
		return dquat(1, 0, 0, 0);
	double oneOverLen = double(1) / len;
	return dquat(q.w * oneOverLen, q.x * oneOverLen, q.y * oneOverLen, q.z * oneOverLen);
}

CybertronCoreAPI dquat conjugate(const dquat& q)
{
	return dquat(q.w, -q.x, -q.y, -q.z);
}

CybertronCoreAPI dquat inverse(const dquat& q)
{
	return conjugate(q) / dot(q, q);
}

CybertronCoreAPI dquat cross(const dquat& q1, const dquat& q2)
{
	return dquat(
		q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
		q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
		q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z,
		q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x);
}

CybertronCoreAPI dquat angleAxis(double angle, const dvec3& v)
{
	dquat Result;

	double const a(angle);
	double const s = std::sin(a * static_cast<double>(0.5));

	Result.w = std::cos(a * static_cast<double>(0.5));
	Result.x = v.x * s;
	Result.y = v.y * s;
	Result.z = v.z * s;
	return Result;
}

CYBERTRON_END
