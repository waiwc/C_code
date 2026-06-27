// 
// Created By: Qilin.Ren 2018/05/29
// 

#pragma once

#include "../DefinesCore.hpp"

CYBERTRON_BEGIN

struct bvec3
{
public:
	bool x, y, z;

	bvec3()
	{
	}

	explicit bvec3(bool f)
	{
		x = f;
		y = f;
		z = f;
	}

	bvec3(bool xx, bool yy, bool zz)
	{
		x = xx;
		y = yy;
		z = zz;
	}

	bvec3(const bvec3& rhs)
	{
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
	}

	~bvec3()
	{
	}

	bool operator == (const bvec3& rhs) const
	{
		return x == rhs.x && y == rhs.y && z == rhs.z;
	}

	bool operator != (const bvec3& rhs) const
	{
		return !(*this == rhs);
	}
};

inline bool* value_ptr(bvec3& m)
{
	return (bool*)(&m.x);
}

inline const bool* value_ptr(const bvec3& m)
{
	return (const bool*)(&m.x);
}

CYBERTRON_END
