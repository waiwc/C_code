#pragma once
#include <iostream>
#include <string>
#include "json.hpp"

typedef nlohmann::json json;
class TrafficObject;
class Trail;

class Task
{
public:
	Task();
	virtual ~Task();

	void Create(float timestamp);

	json& GetAsset()
	{
		return mAsset;
	}

	void Append(TrafficObject* trafficObject);


private:
	json mAsset;
};

class TrafficObject
{
public:
	TrafficObject(const std::string& objectId, const std::string& assetId, const std::string& objectType, double speed, const std::string& trailName, double distance);
	virtual ~TrafficObject();

	json& GetAsset()
	{ return mAsset; }

	std::string& GetName()
	{ return mTrailName; }

	int GetTrailSegmentIndex()
	{ return mTrailSegmentIndex; }
	void SetTrailSegmentIndex(int index)
	{ mTrailSegmentIndex = index; }

	double GetPassedDistance()
	{ return mPassedDistance; }
	void SetPassedDistance(double passedDistance)
	{ mPassedDistance = passedDistance; }

	double GetSpeed()
	{ return mSpeed; }
	void SetSpeed(double speed)
	{ mSpeed = speed; }


	virtual void Update(double x, double y);

protected:
	json mAsset;
	double mPassedDistance;
	int mTrailSegmentIndex;
	double mSpeed;
	std::string mTrailName;
};

class Simulation
{
public:
	Simulation(int socket);
	virtual ~Simulation();

	void Append(Task* task);
	void Run();
	float GetTimstamp()
	{
		return mTimestamp;
	}

protected:
	void Clear();

private:
	int mSocket;
	json mDataPackage;
	float mTimestamp;
};

class Traffic
{
public:
	Traffic();
	virtual ~Traffic();
	void AppendTrail(const std::string& name, Trail* trail);
	void AppendTrafficObject(TrafficObject* trafficObject);
	void UpdateObjects();
	std::vector<TrafficObject*>& GetTrafficObjects()
	{ return mTrafficObjects; }

private:
	std::map<std::string, Trail*> mTrails;
	std::vector<TrafficObject*> mTrafficObjects;
};

class Trail
{
public:
	Trail();
	virtual ~Trail();

	bool Create(const std::string& path);
	double GetTrailLength();
	void GetPosition(TrafficObject* trafficObject, double duration, double& x, double& y, int& segmentIndex, double& distance);
	std::string GetName() { return mName;}

private:
	std::string mName;
	json mWayPoints;
	std::vector<double> mLengths;
	std::vector<double> mSegmentLengths;
	long mWayPointsCount;
};