import socket
import json
import time
import math
import os

class Task:
    def __init__(self):
        self.asset = {}

    def Append(self, trafficObject):
        self.asset['Objects'].append(trafficObject.asset)

    def Create(self, timestamp):
        self.asset['TimeStamp'] =str(timestamp)
        self.asset['Objects'] = []
        return self.asset

class TrafficObject:
    def __init__(self, objectId, assetId, objectType, speed, trailName, distance):
        self.asset = {}
        self.asset['ObjectId'] = objectId
        self.asset['AssetId'] = assetId
        self.asset['ObjectType'] = objectType
        position = {}
        position['X'] = 0
        position['Y'] = 0
        self.asset['Position'] = position
        self.passedDistance = distance
        self.trailSegmentIndex = 0
        self.speed = speed
        self.trailName = trailName

    def Update(self, x, y):
        self.asset['Position']['X'] = x
        self.asset['Position']['Y'] = y

class Simulation:
    def __init__(self, socket):
        self.timestamp = 0
        self.socket = socket
        self.dataPackage = {}
        self.dataPackage['Data'] = []

    def Clear(self):
        self.dataPackage['Data'].clear()

    def Append(self, task):
        self.dataPackage['Data'].append(task.asset)

    def Run(self, traffic):
        sendData = (json.dumps(self.dataPackage)).encode('utf-8')
        self.socket.send(sendData)
        self.timestamp = self.timestamp + 0.1
        self.Clear()

class Traffic:
    def __init__(self):
        self.trails = {}
        self.trafficObjects = []

    def AppendTrail(self, name, trail):
        self.trails[name] = trail
    
    def AppendTrafficObject(self, trafficObject):
        self.trafficObjects.append(trafficObject)

    def UpdateObjects(self):
        for trafficObject in self.trafficObjects:
            trail = self.trails[trafficObject.trailName]
            (x, y, index, distance) = trail.GetPosition(trafficObject, 0.1)
            trafficObject.trailSegmentIndex = index
            trafficObject.passedDistance = distance
            trafficObject.Update(x, y)

class Trail:
    def __init__(self):
        self.name = ""
        self.wayPoints = []
        self.lengths = []
        self.segmentLengths = []
        self.wayPointsCount = len(self.wayPoints)
    
    def Create(self, path):
        fileStream = open(path, "rb")
        self.wayPoints = json.load(fileStream)["waypoints"]
        self.wayPointsCount = len(self.wayPoints)
        length = 0
        for index in range(self.wayPointsCount - 1):
            start = self.wayPoints[index]
            end = self.wayPoints[index + 1]
            distance = math.sqrt((float(end['x']) - float(start['x'])) * (float(end['x']) - float(start['x'])) + (float(end['y']) - float(start['y'])) * (float(end['y']) - float(start['y'])))
            self.segmentLengths.append(distance)
            length = length + distance
            self.lengths.append(length)

    def GetTrailLength(self):
        return self.lengths[len(self.lengths) - 1]

    def GetPosition(self, trafficObject, duration):
        index = trafficObject.trailSegmentIndex
        passedDistance = trafficObject.passedDistance
        speed = trafficObject.speed
        segmentCount = len(self.lengths)
        nextDistance = passedDistance + speed * duration
        nextIndex = index

        while nextIndex < segmentCount:
            if self.lengths[nextIndex] > nextDistance:
                nextIndex = nextIndex -1
                break
            nextIndex = nextIndex + 1
        
        if nextIndex == segmentCount:
            trafficObject.passedDistance = 0
            trafficObject.trailSegmentIndex = 0
            trailOrigin = self.wayPoints[0]
            return (trailOrigin['x'], trailOrigin['y'], 0, 0)
        else:
            if nextIndex < 0:
                leftDistance = nextDistance
            else:
                leftDistance = nextDistance - self.lengths[nextIndex]
            u = leftDistance / self.segmentLengths[nextIndex + 1]
            fromPoint = self.wayPoints[nextIndex + 1]
            toPoint = self.wayPoints[nextIndex + 2]
            x = fromPoint['x'] + u * (toPoint['x'] - fromPoint['x'])
            y = fromPoint['y'] + u * (toPoint['y'] - fromPoint['y'])

            return (x, y, nextIndex + 1, nextDistance)

if __name__ == "__main__":

    trafficProviderSocket = socket.socket()
    trafficProviderSocket.connect(("127.0.0.1", 21568))
    print("start data process node...\n")
    print("connect traffic provider ip: 127.0.0.1 port:21568")

    simulation = Simulation(trafficProviderSocket)
    traffic = Traffic()

    filePath = os.path.realpath(__file__)
    currentDir = os.path.dirname(filePath)

    trail_car = Trail()
    trail_car.Create(os.path.join(currentDir, ".\\vehicleWayPoints.json"))
    traffic.AppendTrail("trail_car", trail_car)

    trail_bike = Trail()
    trail_bike.Create(os.path.join(currentDir, ".\\bikeWayPoints.json"))
    traffic.AppendTrail("trail_bike", trail_bike)

    trail_pedestrian = Trail()
    trail_pedestrian.Create(os.path.join(currentDir, ".\\pedestrianWayPoints.json"))
    traffic.AppendTrail("trail_pedestrian", trail_pedestrian)

    # create cars for trail1
    trafficObjects = []
    carCount = int(trail_car.GetTrailLength() / 20)
    for index in range(carCount):
        car = TrafficObject(str(10000 + index), "1000000", "Vehicle", 20, "trail_car", index * 20)
        traffic.AppendTrafficObject(car)
        trafficObjects.append(car)

    bikeCount = int(trail_bike.GetTrailLength() / 6)
    for index in range(bikeCount):
        bike = TrafficObject(str(30000 + index), "1200000", "Bike", 8, "trail_bike", index * 6)
        traffic.AppendTrafficObject(bike)
        trafficObjects.append(bike)

    pedestrianCount = int(trail_pedestrian.GetTrailLength() / 3)
    for index in range(pedestrianCount):
        pedestrian = TrafficObject(str(20000 + index), "1100001", "Pedestrian", 3, "trail_pedestrian", index * 3)
        traffic.AppendTrafficObject(pedestrian)
        trafficObjects.append(pedestrian)

    while True:

        task = Task()
        task.Create(simulation.timestamp)
        for trafficObject in trafficObjects:
            task.Append(trafficObject)

        simulation.Append(task)
        traffic.UpdateObjects()
        simulation.Run(traffic)
        time.sleep(0.1)