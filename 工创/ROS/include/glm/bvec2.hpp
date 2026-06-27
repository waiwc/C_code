// 
// Created By: Qilin.Ren 2018/05/29
// 

#pragma once

#include "../DefinesCore.hpp"

CYBERTRON_BEGIN

struct bvec2
{
public:
	bool x, y;

	bvec2()
	{
	}

	explicit bvec2(bool f)
	{
		x = f;
		y = f;
	}

	bvec2(bool xx, bool yy)
	{
		x = xx;
		y = yy;
	}

	bvec2(const bvec2& rhs)
	{
		x = rhs.x;
		y = rhs.y;
	}

	~bvec2()
	{
	}

	bool operator == (const bvec2& rhs) const
	{
		return x == rhs.x && y == rhs.y;
	}

	bool operator != (const bvec2& rhs) const
	{
		return !(*this == rhs);
	}
};

inline bool* value_ptr(bvec2& m)
{
	return (bool*)(&m.x);
}

inline const bool* value_ptr(const bvec2& m)
{
	return (const bool*)(&m.x);
}

CYBERTRON_END
