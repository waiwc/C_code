// 
// Created By: Qilin.Ren 2018/05/29
// 

#pragma once

#include "../DefinesCore.hpp"

CYBERTRON_BEGIN

struct ivec3
{
public:
	int x, y, z;

	ivec3()
	{
	}

	explicit ivec3(int f)
	{
		x = f;
		y = f;
		z = f;
	}

	ivec3(int xx, int yy, int zz)
	{
		x = xx;
		y = yy;
		z = zz;
	}

	ivec3(const ivec3& rhs)
	{
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
	}

	~ivec3()
	{
	}

	bool operator == (const ivec3& rhs) const
	{
		return x == rhs.x && y == rhs.y && z == rhs.z;
	}

	bool operator != (const ivec3& rhs) const
	{
		return !(*this == rhs);
	}
};

inline int* value_ptr(ivec3& m)
{
	return (int*)(&m.x);
}

inline const int* value_ptr(const ivec3& m)
{
	return (const int*)(&m.x);
}

inline ivec3 make_ivec3(const int* pValues)
{
	return ivec3(pValues[0], pValues[1], pValues[2]);
}

CYBERTRON_END
