// 
// Created By: Qilin.Ren 2018/05/29
// 

#include "glm/quat.hpp"
#include "glm/mat4.hpp"
#include "glm/mat3.hpp"
#include "glm/vec3.hpp"
#include "glm/vec4.hpp"
#include <assert.h>
#include <cmath>
#include <float.h>

CYBERTRON_BEGIN

static float mixf(float val1, float val2, float a)
{
	return val1 * (1 - a) + val2 * a;
}

CybertronCoreAPI quat::quat(const mat3& m)
{
	*this = quat_cast(m);
}

CybertronCoreAPI quat::quat(const mat4& m)
{
	*this = quat_cast(m);
}

CybertronCoreAPI quat::quat(const vec3& u, const vec3& v)
{
	vec3 const LocalW(cross(u, v));
	float Dot = dot(u, v);
	quat q(float(1) + Dot, LocalW.x, LocalW.y, LocalW.z);

	*this = normalize(q);
}

CybertronCoreAPI quat::quat(float angleRadians, const vec3& unitAxis)
{
    //assert(std::abs(1.0f - unitAxis.magnitude()) < 1e-3f);
    const float a = angleRadians * 0.5f;
    const float s = std::sin(a);
    w = std::cos(a);
    x = unitAxis.x * s;
    y = unitAxis.y * s;
    z = unitAxis.z * s;
}

vec3 cos(const vec3& angles)
{
	return vec3(
		cosf(angles.x),
		cosf(angles.y),
		cosf(angles.z));
}

vec3 sin(const vec3& angles)
{
	return vec3(
		sinf(angles.x),
		sinf(angles.y),
		sinf(angles.z));
}

CybertronCoreAPI quat::quat(const vec3& eulerAngle)
{
	vec3 c = cos(eulerAngle * float(0.5));
	vec3 s = sin(eulerAngle * float(0.5));

	this->w = c.x * c.y * c.z + s.x * s.y * s.z;
	this->x = s.x * c.y * c.z - c.x * s.y * s.z;
	this->y = c.x * s.y * c.z + s.x * c.y * s.z;
	this->z = c.x * c.y * s.z - s.x * s.y * c.z;
}

CybertronCoreAPI quat quat::operator * (const quat& rhs) const
{
	quat lhs(*this);
	lhs *= rhs;
	return lhs;
}

CybertronCoreAPI quat quat::operator * (float rhs) const
{
	return quat(w * rhs, x * rhs, y * rhs, z * rhs);
}

CybertronCoreAPI quat quat::operator / (float rhs) const
{
	return quat(w / rhs, x / rhs, y / rhs, z / rhs);
}

CybertronCoreAPI quat quat::operator + (const quat& rhs) const
{
	return quat(
		w + rhs.w,
		x + rhs.x,
		y + rhs.y,
		z + rhs.z);
}

CybertronCoreAPI quat quat::operator - (const quat& rhs) const
{
	return quat(
		w - rhs.w,
		x - rhs.x,
		y - rhs.y,
		z - rhs.z);
}

CybertronCoreAPI quat quat::operator - () const
{
	const quat& q = *this;
	return quat(-q.w, -q.x, -q.y, -q.z);
}

CybertronCoreAPI quat& quat::operator *= (const quat& r)
{
	quat const p(*this);
	quat const q(r);

	this->w = p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z;
	this->x = p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y;
	this->y = p.w * q.y + p.y * q.w + p.z * q.x - p.x * q.z;
	this->z = p.w * q.z + p.z * q.w + p.x * q.y - p.y * q.x;
	return *this;
}

CybertronCoreAPI vec3 quat::operator * (const vec3& v) const
{
	const quat& q = *this;
	vec3 const QuatVector(q.x, q.y, q.z);
	vec3 const uv(cross(QuatVector, v));
	vec3 const uuv(cross(QuatVector, uv));

	return v + ((uv * q.w) + uuv) * static_cast<float>(2);
}

CybertronCoreAPI quat operator * (float lhs, const quat& rhs)
{
	return quat(lhs * rhs.w, lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
}

CybertronCoreAPI quat quat_cast(const mat3& m)
{
	float fourXSquaredMinus1 = m[0][0] - m[1][1] - m[2][2];
	float fourYSquaredMinus1 = m[1][1] - m[0][0] - m[2][2];
	float fourZSquaredMinus1 = m[2][2] - m[0][0] - m[1][1];
	float fourWSquaredMinus1 = m[0][0] + m[1][1] + m[2][2];

	int biggestIndex = 0;
	float fourBiggestSquaredMinus1 = fourWSquaredMinus1;
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

	float biggestVal = sqrtf(fourBiggestSquaredMinus1 + float(1)) * float(0.5);
	float mult = static_cast<float>(0.25) / biggestVal;

	quat Result;
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

CybertronCoreAPI quat quat_cast(const mat4& m)
{
	return quat_cast(mat3(m));
}

CybertronCoreAPI mat3 mat3_cast(const quat& q)
{
	mat3 Result(float(1));
	float qxx(q.x * q.x);
	float qyy(q.y * q.y);
	float qzz(q.z * q.z);
	float qxz(q.x * q.z);
	float qxy(q.x * q.y);
	float qyz(q.y * q.z);
	float qwx(q.w * q.x);
	float qwy(q.w * q.y);
	float qwz(q.w * q.z);

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

CybertronCoreAPI mat4 mat4_cast(const quat& q)
{
	return mat4(mat3_cast(q));
}

CybertronCoreAPI float dot(const quat& x, const quat& y)
{
	vec4 tmp(x.x * y.x, x.y * y.y, x.z * y.z, x.w * y.w);
	return (tmp.x + tmp.y) + (tmp.z + tmp.w);
}

CybertronCoreAPI float length(const quat& q)
{
	return sqrtf(dot(q, q));
}

CybertronCoreAPI quat lerp(const quat& x, const quat& y, float a)
{
	// Lerp is only defined in [0, 1]
	assert(a >= static_cast<float>(0));
	assert(a <= static_cast<float>(1));

	return x * (float(1) - a) + (y * a);
}

CybertronCoreAPI quat slerp(const quat& x, const quat& y, float a)
{
	quat z = y;

	float cosTheta = dot(x, y);

	// If cosTheta < 0, the interpolation will take the long way around the sphere. 
	// To fix this, one quat must be negated.
	if (cosTheta < float(0))
	{
		z = -y;
		cosTheta = -cosTheta;
	}

	// Perform a linear interpolation when cosTheta is close to 1 to avoid side effect of sin(angle) becoming a zero denominator
	if (cosTheta > float(1) - FLT_EPSILON)
	{
		// Linear interpolation
		return quat(
			mixf(x.w, z.w, a),
			mixf(x.x, z.x, a),
			mixf(x.y, z.y, a),
			mixf(x.z, z.z, a));
	}
	else
	{
		// Essential Mathematics, page 467
		float angle = acosf(cosTheta);
		return (sinf((float(1) - a) * angle) * x + sinf(a * angle) * z) / sinf(angle);
	}
}

CybertronCoreAPI float roll(quat const & q)
{
	return float(atan2f(float(2) * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z));
}

CybertronCoreAPI float pitch(quat const & q)
{
	return float(atan2f(float(2) * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z));
}

inline float clamp(float x, float minVal, float maxVal)
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

CybertronCoreAPI float yaw(quat const & q)
{
	return asinf(clamp(float(-2) * (q.x * q.z - q.w * q.y), -1.0f, 1.0f));
}

CybertronCoreAPI vec3 eulerAngles(const quat& x)
{
	// NOTE: Definition of pitch/yaw/roll coule be different based on scenario.
	//       Just take the three parameters as rotation of X, Y and Z axis.
	return vec3(pitch(x), yaw(x), roll(x));
}

CybertronCoreAPI mat4 eulerAngleXYZ(float t1, float t2, float t3)
{
	float c1 = cosf(-t1);
	float c2 = cosf(-t2);
	float c3 = cosf(-t3);
	float s1 = sinf(-t1);
	float s2 = sinf(-t2);
	float s3 = sinf(-t3);

	mat4 Result;
	Result[0][0] = c2 * c3;
	Result[0][1] = -c1 * s3 + s1 * s2 * c3;
	Result[0][2] = s1 * s3 + c1 * s2 * c3;
	Result[0][3] = static_cast<float>(0);
	Result[1][0] = c2 * s3;
	Result[1][1] = c1 * c3 + s1 * s2 * s3;
	Result[1][2] = -s1 * c3 + c1 * s2 * s3;
	Result[1][3] = static_cast<float>(0);
	Result[2][0] = -s2;
	Result[2][1] = s1 * c2;
	Result[2][2] = c1 * c2;
	Result[2][3] = static_cast<float>(0);
	Result[3][0] = static_cast<float>(0);
	Result[3][1] = static_cast<float>(0);
	Result[3][2] = static_cast<float>(0);
	Result[3][3] = static_cast<float>(1);
	return Result;
}

CybertronCoreAPI quat normalize(const quat& q)
{
	float len = length(q);
	if (len <= float(0)) // Problem
		return quat(1, 0, 0, 0);
	float oneOverLen = float(1) / len;
	return quat(q.w * oneOverLen, q.x * oneOverLen, q.y * oneOverLen, q.z * oneOverLen);
}

CybertronCoreAPI quat conjugate(const quat& q)
{
	return quat(q.w, -q.x, -q.y, -q.z);
}

CybertronCoreAPI quat inverse(const quat& q)
{
	return conjugate(q) / dot(q, q);
}

CybertronCoreAPI quat cross(const quat& q1, const quat& q2)
{
	return quat(
		q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
		q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
		q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z,
		q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x);
}

CybertronCoreAPI quat angleAxis(float angle, const vec3& v)
{
	quat Result;

	float const a(angle);
	float const s = sinf(a * static_cast<float>(0.5));

	Result.w = cosf(a * static_cast<float>(0.5));
	Result.x = v.x * s;
	Result.y = v.y * s;
	Result.z = v.z * s;
	return Result;
}

CybertronCoreAPI vec3 rotate(const quat& q, const vec3& v)
{
    const float vx = 2.0f * v.x;
    const float vy = 2.0f * v.y;
    const float vz = 2.0f * v.z;
    const float w2 = q.w * q.w - 0.5f;
    const float dot2 = (q.x * vx + q.y * vy + q.z * vz);
    return cybertron::vec3((vx * w2 + (q.y * vz - q.z * vy) * q.w + q.x * dot2), 
        (vy * w2 + (q.z * vx - q.x * vz) * q.w + q.y * dot2),
        (vz * w2 + (q.x * vy - q.y * vx) * q.w + q.z * dot2));
}


CybertronCoreAPI bool isFinite(const quat& q)
{
    return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}

/**
\brief returns true if finite and magnitude is close to unit
*/
CybertronCoreAPI bool isUnit(const quat& q)
{
    const float unitTolerance = 1e-4f;
    return isFinite(q) && std::abs(length(q) - 1) < unitTolerance;
}

/**
\brief returns true if finite and magnitude is reasonably close to unit to allow for some accumulation of error vs
isValid
*/
CybertronCoreAPI bool isSane(const quat& q)
{
    const float unitTolerance = 1e-2f;
    return isFinite(q) && std::abs(length(q) - 1) < unitTolerance;
}

CybertronCoreAPI const vec3 rotateInv(const quat& q, const vec3& v)
{
    const float vx = 2.0f * v.x;
    const float vy = 2.0f * v.y;
    const float vz = 2.0f * v.z;
    const float w2 = q.w * q.w - 0.5f;
    const float dot2 = (q.x * vx + q.y * vy + q.z * vz);
    return vec3((vx * w2 - (q.y * vz - q.z * vy) * q.w + q.x * dot2), 
        (vy * w2 - (q.z * vx - q.x * vz) * q.w + q.y * dot2),
        (vz * w2 - (q.x * vy - q.y * vx) * q.w + q.z * dot2));
}

CYBERTRON_END
