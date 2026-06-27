/*!
* Utility functions for calculation
*/

#pragma once

#include <cmath>

class UtilMath
{
public:
	static double calculateSpeed(const double velX, const double velY, const double velZ)
	{
		return std::sqrt(std::pow(velX, 2) + std::pow(velY, 2) + std::pow(velZ, 2));
	}
	static float mpsToKmph(float s)
	{
		return s * 3.6f;
	}
	static float kmphToMps(float s)
	{
		return s / 3.6f;
	}
};

class LookupTable2D
{
public:
	LookupTable2D() {}
	~LookupTable2D() {}
	void addPair(const float x, const float y) {
		mDataPairs.push_back(std::make_pair(x, y));
	}

	float getYVal(const float x) {
		if (mDataPairs.empty()) {
			return 0.f;
		}

		if (1 == mDataPairs.size()) {
			return getY(0);
		}
	
		float val = 0.f;
		if (x >= getX(mDataPairs.size() - 1)) {
			val = getY(mDataPairs.size() - 1);
		}
		else if (x <= getX(0)) {
			val = getY(0);
		}
		else {
			for (int i = 1; i < static_cast<int>(mDataPairs.size()); ++i) {
				if (x < getX(i)) {
					val = getY(i - 1);
					break;
				}
			}
		}
	}
	float getY(const int i) { return mDataPairs[i].second; }
	float getX(const int i) { return mDataPairs[i].first; }

private:
	std::vector<std::pair<float, float>> mDataPairs;
};
