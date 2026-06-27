#include "pch.h"
#include "main.h"
#include "Utils.h"
#include <fstream>

#define TIME_STEP 0.1f
#define VEHICLE_TYPR "Vehicle"
#define VIKE_TYPR "Bike"
#define PEDESTRIAN_TYPR "Pedestrian"



// Task
Task::Task()
{
}
Task::~Task()
{
}
void Task::Create(float timestamp)
{
	mAsset["TimeStamp"] = std::to_string(timestamp);
	mAsset["Objects"] = json();
}
void Task::Append(TrafficObject* trafficObject)
{
	mAsset["Objects"].push_back(trafficObject->GetAsset());
}

// TrafficObject
TrafficObject::TrafficObject(const std::string& objectId, const std::string& assetId, const std::string& objectType, double speed, const std::string& trailName, double distance)
{
	mAsset["ObjectId"] = objectId;
	mAsset["AssetId"] = assetId;
	mAsset["ObjectType"] = objectType;
	json position;
	position["X"] = 0;
	position["Y"] = 0;
	mAsset["Position"] = position;
	mPassedDistance = distance;
	mTrailSegmentIndex = 0;
	mSpeed = speed;
	mTrailName = trailName;
}

TrafficObject::~TrafficObject()
{
}

void TrafficObject::Update(double x, double y)
{
	mAsset["Position"]["X"] = x;
	mAsset["Position"]["Y"] = y;
}

// Simulation
Simulation::Simulation(int socket)
{
	mTimestamp = 0;
	mSocket = socket;
	mDataPackage["Data"] = json();
}

Simulation::~Simulation()
{
}

void Simulation::Append(Task* task)
{
	mDataPackage["Data"].push_back(task->GetAsset());
}

void Simulation::Run()
{
	Utils::SendDataBySocket(mSocket, mDataPackage.dump());
	mTimestamp = mTimestamp + TIME_STEP;
	Clear();
}

void Simulation::Clear()
{
	mDataPackage["Data"].clear();
}

// Traffic
Traffic::Traffic()
{
}

Traffic::~Traffic()
{
	for (std::vector<TrafficObject*>::iterator it = mTrafficObjects.begin(); it != mTrafficObjects.end(); it++)
	{
		delete *it;
		*it = NULL;
	}
	mTrafficObjects.clear();
	for (std::map<std::string, Trail*>::iterator it = mTrails.begin(); it != mTrails.end(); it++)
	{
		delete it->second;
		it->second = NULL;
	}
	mTrails.clear();
}

void Traffic::AppendTrail(const std::string& name, Trail* trail)
{
	mTrails[name] = trail;
}

void Traffic::AppendTrafficObject(TrafficObject* trafficObject)
{
	mTrafficObjects.push_back(trafficObject);
}

void Traffic::UpdateObjects()
{
	for (size_t i = 0; i < mTrafficObjects.size(); i++)
	{
		TrafficObject* trafficObject = mTrafficObjects[i];
		if (mTrails.find(trafficObject->GetName()) == mTrails.end())
		{
			continue;
		}
		Trail* trail = mTrails[trafficObject->GetName()];
		double x = 0;
		double y = 0;
		int segmentIndex = 0;
		double distance = 0;
		trail->GetPosition(trafficObject, TIME_STEP, x, y, segmentIndex, distance);
		trafficObject->SetTrailSegmentIndex(segmentIndex);
		trafficObject->SetPassedDistance(distance);
		trafficObject->Update(x, y);
	}
}


// Trail
Trail::Trail()
{
	mWayPoints = json();
}

Trail::~Trail()
{
}

bool Trail::Create(const std::string& path)
{
	std::cout << "path : " << path.c_str() << std::endl;
	std::ifstream trailStream(path);
	if (!trailStream.good())
	{
		std::cout << "Trail.Create fail : " << path.c_str() << std::endl;
		return false;
	}
	std::stringstream trailBuffer;
	trailBuffer << trailStream.rdbuf();
	std::string trailContent(trailBuffer.str());

	try
	{
		mWayPoints = json::parse(trailContent)["waypoints"];
		mWayPointsCount = (int)(mWayPoints.size());
		double length = 0;
		for (int index = 0; index < mWayPointsCount - 1; index++)
		{
			json& start = mWayPoints[index];
			json& end = mWayPoints[index + 1];
			double distance = sqrt((float(end["x"].get<double>()) - float(start["x"].get<double>())) * (float(end["x"].get<double>()) - float(start["x"].get<double>())) + (float(end["y"].get<double>()) - float(start["y"].get<double>())) * (float(end["y"].get<double>()) - float(start["y"].get<double>())));
			mSegmentLengths.push_back(distance);
			length = length + distance;
			mLengths.push_back(length);
		}
	}
	catch (const std::exception&)
	{
		std::cout << "Trail.Create.parse fail : " << std::endl;
		return false;
	}

	return true;
}

double Trail::GetTrailLength()
{
	return mLengths[mLengths.size() - 1];
}

void Trail::GetPosition(TrafficObject* trafficObject, double duration, double& x, double& y, int& segmentIndex, double& distance)
{
	int index = trafficObject->GetTrailSegmentIndex();
	double passedDistance = trafficObject->GetPassedDistance();
	double speed = trafficObject->GetSpeed();
	int segmentCount = (int)mLengths.size();
	double nextDistance = passedDistance + speed * duration;
	int nextIndex = index;

	while (nextIndex < segmentCount)
	{
		if (mLengths[nextIndex] > nextDistance)
		{
			nextIndex = nextIndex - 1;
			break;
		}
		nextIndex = nextIndex + 1;
	}

	if (nextIndex == segmentCount)
	{
		trafficObject->SetPassedDistance(0);
		trafficObject->SetTrailSegmentIndex(0);
		json& trailOrigin = mWayPoints[0];
		x = trailOrigin["x"].get<double>();
		y = trailOrigin["y"].get<double>();
		segmentIndex = 0;
		distance = 0;
	}
	else
	{
		double leftDistance = 0;
		if (nextIndex < 0)
		{
			leftDistance = nextDistance;
		}
		else
		{
			leftDistance = nextDistance - mLengths[nextIndex];
		}
		double u = leftDistance / mSegmentLengths[nextIndex + 1];
		json& fromPoint = mWayPoints[nextIndex + 1];
		json& toPoint = mWayPoints[nextIndex + 2];
		x = fromPoint["x"].get<double>() + u * (toPoint["x"].get<double>() - fromPoint["x"].get<double>());
		y = fromPoint["y"].get<double>() + u * (toPoint["y"].get<double>() - fromPoint["y"].get<double>());
		segmentIndex = nextIndex + 1;
		distance = nextDistance;
	}
}

int main(int argc, char* argv[])
{
	/*std::cout << "argv[0]  = " << argv[0] << std::endl;
	if (argc <= 1)
	{
		return 0;
	}*/

	std::string projectPath = "../../../../Traffic/Python";//argv[1];
	//std::cout << "projectPath  = " << projectPath << std::endl;

	int trafficProviderSocket = Utils::CreateTrafficProviderSocket();
	std::cout << "start data process node..." << std::endl;
	std::cout << "connect traffic provider ip: 127.0.0.1 port:21568" << std::endl;

	Simulation simulation(trafficProviderSocket);

	Traffic traffic;

	//Trail e2w;
	//bool succeed = e2w.Create(projectPath + "/highpoint.json");
	//if (succeed == false)
	//{
	//	return 0;
	//}
	//traffic.AppendTrail("highpoint", &e2w);

	//int carCount = int(e2w.GetTrailLength() / 20);
	//for (int index = 0; index < carCount; index++)
	//{
	//	TrafficObject* car = new TrafficObject(std::to_string(10000 + index), "1000000", VEHICLE_TYPR, 20, "highpoint", index * 20);
	//	traffic.AppendTrafficObject(car);
	//}


	Trail e2w;
	bool succeed = e2w.Create(projectPath + "/vehicleWayPoints.json");
	if (succeed == false)
	{
		return 0;
	}
	traffic.AppendTrail("vehicleWayPoints", &e2w);


	Trail w2e;
	succeed = w2e.Create(projectPath + "/bikeWayPoints.json");
	if (succeed == false)
	{
		return 0;
	}
	traffic.AppendTrail("bikeWayPoints", &w2e);

	Trail trailpedestrian;
	succeed = trailpedestrian.Create(projectPath + "/pedestrianWayPoints.json");
	if (succeed == false)
	{
		return 0;
	}
	traffic.AppendTrail("pedestrianWayPoints", &trailpedestrian);


	int carCount = int(e2w.GetTrailLength() / 20);
	for (int index = 0; index < carCount; index++)
	{
		TrafficObject* car = new TrafficObject(std::to_string(10000 + index), "1000000", VEHICLE_TYPR, 20, "vehicleWayPoints", index * 20);
		traffic.AppendTrafficObject(car);
	}


	int bikeCount = int(w2e.GetTrailLength() / 6);
	for (int index = 0; index < bikeCount; index++)
	{
		TrafficObject* bike = new TrafficObject(std::to_string(20000 + index), "1200000", VIKE_TYPR, 8, "bikeWayPoints", index * 6);
		traffic.AppendTrafficObject(bike);
	}

	int pedestrianCount = int(trailpedestrian.GetTrailLength() / 3);
	for (int index = 0; index < pedestrianCount; index++)
	{
		TrafficObject* pedestrian = new TrafficObject(std::to_string(30000 + index), "1100001", PEDESTRIAN_TYPR, 3, "pedestrianWayPoints", index * 3);
		traffic.AppendTrafficObject(pedestrian);
	}

	while (true)
	{
		Task task;
		task.Create(simulation.GetTimstamp());
		std::vector<TrafficObject*>& trafficObjects = traffic.GetTrafficObjects();
		for (size_t index = 0; index < trafficObjects.size(); index++)
		{
			TrafficObject* trafficObject = trafficObjects[index];
			task.Append(trafficObject);
		}
		simulation.Append(&task);
		traffic.UpdateObjects();
		simulation.Run();

#if defined(__linux__)
		sleep(TIME_STEP);
#else
		Sleep((DWORD)(TIME_STEP * 1000));
#endif
	}
}
