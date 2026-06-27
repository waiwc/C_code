#ifndef TRANS_H_
#define TRANS_H_

#define DEBUG_CONTROL

#include "Pub.h"
#include "Sub.h"

#include <fastrtps/Domain.h>
#include <fastrtps/log/Log.h>

// SimOneAPI
#include "Service/SimOneIOStruct.h"
#include "SimOneServiceAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneSensorAPI.h"
#include "SimOneHDMapAPI.h"
#include "SimOneStreamingAPI.h"
// Common
#include "common/verify_ip.hpp"
#include "common/Config.h"

using namespace eprosima;
using namespace fastrtps;
using namespace rtps;

class Trans
{
public:
    Trans();
    ~Trans();

    bool PramOpt(int argc, char *argv[]);

    void ConfigIni();
    bool SimOneIni();
    bool PubIni();
    bool SubIni();
    bool Init();

    static void CB_PubGps(const char *mainVehicleId, SimOne_Data_Gps *pGps);
    static void CB_PubObstacle(const char *mainVehicleId, SimOne_Data_Obstacle *pObstacle);
    void MonitorCaseStatus();
    void Pub();
    void Sub();
    void Run();

private:
    int mDomain;
    int mTransport; // 1: udp 2: tcp 3: udp6 4: tcp6
    ReliabilityQosPolicyKind mKind; // RELIABLE_RELIABILITY_QOS; BEST_EFFORT_RELIABILITY_QOS
    std::string mSub_TCP_Ip;
    int mSub_TCP_PortBase;
    int mPub_TCP_PortBase;
    std::string mTopicGps;
    std::string mTopicObstacle;
    std::string mTopicControl;

    static BenchMarkPublisher* mpPubGps;
    static BenchMarkPublisher* mpPubObstacle;
#ifdef DEBUG_CONTROL
    static BenchMarkPublisher* mpPubControl;
#endif
    BenchMarkSubscriber* mpSubControl;

    // SimOneAPI
    std::string mBridgeIO_Ip;
    std::string mVehicle_Id;
    SimOne_Data_CaseInfo mCaseInfo;
    bool mStreamingEnabled;
};

#endif