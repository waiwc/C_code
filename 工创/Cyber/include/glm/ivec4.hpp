// 
// Created By: Qilin.Ren 2018/05/29
// 

#pragma once

#include "../DefinesCore.hpp"

CYBERTRON_BEGIN

struct ivec4
{
public:
	int x, y, z, w;

	ivec4()
	{
	}

	explicit ivec4(int f)
	{
		x = f;
		y = f;
		z = f;
		w = f;
	}

	ivec4(int xx, int yy, int zz, int ww)
	{
		x = xx;
		y = yy;
		z = zz;
		w = ww;
	}

	ivec4(const ivec4& rhs)
	{
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		w = rhs.w;
	}

	~ivec4()
	{
	}

	bool operator == (const ivec4& rhs) const
	{
		return x == rhs.x && y == rhs.y && z == rhs.z && w == rhs.w;
	}

	bool operator != (const ivec4& rhs) const
	{
		return !(*this == rhs);
	}
};

inline int* value_ptr(ivec4& m)
{
	return (int*)(&m.x);
}

inline const int* value_ptr(const ivec4& m)
{
	return (const int*)(&m.x);
}

inline ivec4 make_ivec4(const int* pValues)
{
	return ivec4(pValues[0], pValues[1], pValues[2], pValues[3]);
}

CYBERTRON_END
