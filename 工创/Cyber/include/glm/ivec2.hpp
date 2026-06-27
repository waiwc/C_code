// 
// Created By: Qilin.Ren 2018/05/29
// 

#pragma once

#include "../DefinesCore.hpp"

CYBERTRON_BEGIN

struct ivec2
{
public:
	int x, y;

	ivec2()
	{
	}

	explicit ivec2(int f)
	{
		x = f;
		y = f;
	}

	ivec2(int xx, int yy)
	{
		x = xx;
		y = yy;
	}

	ivec2(const ivec2& rhs)
	{
		x = rhs.x;
		y = rhs.y;
	}

	~ivec2()
	{
	}

	bool operator == (const ivec2& rhs) const
	{
		return x == rhs.x && y == rhs.y;
	}

	bool operator != (const ivec2& rhs) const
	{
		return !(*this == rhs);
	}

	ivec2 operator - (const ivec2& rhs) const
	{
		return ivec2(x - rhs.x, y - rhs.y);
	}

	ivec2 operator + (const ivec2& rhs) const
	{
		return ivec2(x + rhs.x, y + rhs.y);
	}

	ivec2 operator * (int f) const
	{
		return ivec2(x * f, y * f);
	}

	ivec2 operator / (int f) const
	{
		return ivec2(x / f, y / f);
	}
};

inline int* value_ptr(ivec2& m)
{
	return (int*)(&m.x);
}

inline const int* value_ptr(const ivec2& m)
{
	return (const int*)(&m.x);
}

inline ivec2 make_ivec2(const int* pValues)
{
	return ivec2(pValues[0], pValues[1]);
}

CYBERTRON_END
