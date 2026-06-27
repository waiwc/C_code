
#include "coord_conver/dvec3.h"
#include <memory>
#include <cmath>

const double EPSILON = 1E-6;
const double MaxValue = 1E+10;


dvec3 dvec3::zero = dvec3(0, 0, 0);
dvec3 dvec3::axisX = dvec3(1, 0, 0);
dvec3 dvec3::axisY = dvec3(0, 1, 0);
dvec3 dvec3::axisZ = dvec3(0, 0, 1);

dvec3::dvec3()
    :
    x(0),
    y(0),
    z(0)
{
}
dvec3::dvec3(const double& xx, const double& yy, const double& zz)
    :
    x(xx),
    y(yy),
    z(zz)
{
}

dvec3::dvec3(const dvec3& rhs)
{
    x = rhs.x;
    y = rhs.y;
    z = rhs.z;
}

bool dvec3::isValidDirection()
{
    return length() > 0.00001;
}

dvec3 dvec3::normalize()
{
    return std::move(normalize(*this));
}

double dvec3::length()
{
    return length(*this);
}

double dvec3::dot(const dvec3& rhs)
{
    return dot(*this, rhs);
}

dvec3 dvec3::cross(const dvec3& rhs)
{
    return std::move(cross(*this, rhs));
}

dvec3 dvec3::normalize(const dvec3& v)
{
    double l = length(v);
    if (l <= EPSILON)
    {
        return dvec3(1, 0, 0);
    }
    return v / l;
}

double dvec3::length(const dvec3& v)
{
    return sqrt(dot(v, v));
}

double dvec3::dot(const dvec3& lhs, const dvec3& rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

dvec3 dvec3::cross(const dvec3& lhs, const dvec3& rhs)
{
    return std::move(dvec3(lhs.y * rhs.z - rhs.y * lhs.z,
        lhs.z * rhs.x - rhs.z * lhs.x,
        lhs.x * rhs.y - rhs.x * lhs.y));
}

double dvec3::angleBetween(const dvec3& lhs, const dvec3& rhs)
{
    double dotVal = dot(normalize(lhs), normalize(rhs));
    if (fabs(dotVal) > 1)
    {
        dotVal = dotVal > 0 ? 1 : -1;
    }
    return acos(dotVal);
}

dvec3 operator +(const dvec3& lhs, const dvec3& rhs)
{
    return std::move(dvec3(
        lhs.x + rhs.x,
        lhs.y + rhs.y,
        lhs.z + rhs.z));
}

dvec3 operator -(const dvec3& lhs, const dvec3& rhs)
{
    return std::move(dvec3(
        lhs.x - rhs.x,
        lhs.y - rhs.y,
        lhs.z - rhs.z));
}

dvec3 operator *(const double& lhs, const dvec3& rhs)
{
    return std::move(dvec3(
        lhs * rhs.x,
        lhs * rhs.y,
        lhs * rhs.z));
}

dvec3 operator *(const dvec3& self, const double& s)
{
    return std::move(dvec3(self.x * s, self.y * s, self.z * s));
}

dvec3 operator /(const dvec3& lhs, const double& rhs)
{
    return std::move(dvec3(
        lhs.x / rhs,
        lhs.y / rhs,
        lhs.z / rhs));
}

bool operator == (const dvec3& lhs, const dvec3& rhs)
{
    if (fabs(lhs.x - rhs.x) > 1E-6 ||
        fabs(lhs.y - rhs.y) > 1E-6 ||
        fabs(lhs.z - rhs.z) > 1E-6)
    {
        return false;
    }

    return true;
}

bool operator !=(const dvec3& lhs, const dvec3& rhs)
{
    if (fabs(lhs.x - rhs.x) > 1E-6 ||
        fabs(lhs.y - rhs.y) > 1E-6 ||
        fabs(lhs.z - rhs.z) > 1E-6)
    {
        return true;
    }

    return false;
}

double dvec3::distance(const dvec3& v1, const dvec3& v2)
{
    return length(v1 - v2);
}
