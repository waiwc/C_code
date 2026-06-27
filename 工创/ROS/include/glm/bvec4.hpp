// 
// Created By: Qilin.Ren 2018/05/29
// 

#pragma once

#include "../DefinesCore.hpp"

CYBERTRON_BEGIN

struct bvec4
{
public:
	bool x, y, z, w;

	bvec4()
	{
	}

	explicit bvec4(bool f)
	{
		x = f;
		y = f;
		z = f;
		w = f;
	}

	bvec4(bool xx, bool yy, bool zz, bool ww)
	{
		x = xx;
		y = yy;
		z = zz;
		w = ww;
	}

	bvec4(const bvec4& rhs)
	{
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		w = rhs.w;
	}

	~bvec4()
	{
	}

	bool operator == (const bvec4& rhs) const
	{
		return x == rhs.x && y == rhs.y && z == rhs.z && w == rhs.w;
	}

	bool operator != (const bvec4& rhs) const
	{
		return !(*this == rhs);
	}
};

inline bool* value_ptr(bvec4& m)
{
	return (bool*)(&m.x);
}

inline const bool* value_ptr(const bvec4& m)
{
	return (const bool*)(&m.x);
}

CYBERTRON_END
