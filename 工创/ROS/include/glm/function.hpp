// 
// Created By: Qilin.Ren 2018/06/22
// 

#pragma once

#include "../DefinesCore.hpp"

CYBERTRON_BEGIN

#ifndef DBL_PI
#	define DBL_PI 3.14159265358979323846
#endif

#ifndef FLT_PI
#	define FLT_PI 3.14159265f
#endif

inline float mix(float val1, float val2, float a)
{
	return val1 * (1 - a) + val2 * a;
}

inline double mix(double val1, double val2, double a)
{
	return val1 * (1 - a) + val2 * a;
}

inline double mix(double val1, double val2, float a)
{
	return val1 * (1 - a) + val2 * a;
}

inline float degreeToRadian(float deg)
{
	return deg * FLT_PI / 180;
}

inline double degreeToRadian(double deg)
{
	return deg * DBL_PI / 180;
}

inline float radianToDegree(float rad)
{
	return rad * 180 / FLT_PI;
}

inline double radianToDegree(double rad)
{
	return rad * 180 / DBL_PI;
}

CYBERTRON_END
