#include "Trans.h"

BenchMarkPublisher* Trans::mpPubGps = nullptr;
BenchMarkPublisher* Trans::mpPubObstacle = nullptr;
#ifdef DEBUG_CONTROL
BenchMarkPublisher* Trans::mpPubControl = nullptr;
#endif

Trans::Trans()
{
    mpPubGps = new BenchMarkPublisher();
    mpPubObstacle = new BenchMarkPublisher();
#ifdef DEBUG_CONTROL
    mpPubControl = new BenchMarkPublisher();
#endif
    mpSubControl = new BenchMarkSubscriber();
}

Trans::~Trans()
{
    delete mpPubGps;
    delete mpPubObstacle;
#ifdef DEBUG_CONTROL
    delete mpPubControl;
#endif
    delete mpSubControl;
    Domain::stopAll();
    Log::Reset();
}

void Trans::ConfigIni()
{
	rr::RrConfig config;
	config.ReadConfig("config.ini");
  if (mBridgeIO_Ip.empty())
  {
	  mBridgeIO_Ip = config.ReadString("BridgeIO", "BridgeIO_IP", "");
  }
  mVehicle_Id = config.ReadString("HostVehicle", "Vehicle_ID", "0");

  mDomain = config.ReadInt("DDS", "Domain", 0);
  mTransport = config.ReadInt("DDS", "Transport", 1); // 1: udp 2: tcp 3: udp6 4: tcp6 Other: sm
  mSub_TCP_Ip = config.ReadString("DDS", "Sub_TCP_IP", "127.0.0.1");
  mSub_TCP_PortBase = config.ReadInt("DDS", "Sub_TCP_PORTBASE", 5102);
  mPub_TCP_PortBase = config.ReadInt("DDS", "Pub_TCP_PORTBASE", 5100);
  if (config.ReadInt("DDS", "Kind", 0) > 0)
  {
      mKind = BEST_EFFORT_RELIABILITY_QOS;
  }
  else
  {
      mKind = RELIABLE_RELIABILITY_QOS;
  }
  mTopicGps = config.ReadString("DDS", "W_Topic_GPS", "BenchMarkTopicTCP");
  mTopicObstacle = config.ReadString("DDS", "W_Topic_Obstacle", "Obstacle");
  mTopicControl = config.ReadString("DDS", "W_Topic_Control", "Control");

  if (config.ReadInt("Sensor", "ENABLED", 0) > 0)
  {
    mStreamingEnabled = true;
  }
  else
  {
    mStreamingEnabled = false;
  }

  // float lat = config.ReadFloat("MapBase", "BASE_LATITUDE", 0);
  // float log = config.ReadFloat("MapBase", "BASE_LONGITUDE", 0);
  // float alt = config.ReadFloat("MapBase", "BASE_ALTITUDE", 0);
}

bool Trans::SimOneIni()
{
  printf("Connecting BridgeIO With Ip : %s ...\n", mBridgeIO_Ip.c_str());

  if (!SimOneAPI::InitSimOneAPI(mVehicle_Id.c_str(), false, mBridgeIO_Ip.c_str()))
  {
    printf("[simone_ini] SimOneAPI::InitSimOneAPI Failed!\n");
    return false;
  }

  int timeout = 30;
  if (!SimOneAPI::LoadHDMap(timeout))
  {
    printf("\033[1m\033[31m[simone_ini] SimOneAPI::LoadHDMap Failed!\033[0m\n");
  }

  // ESimOne_Data_Vehicle_State index[INDEX_SIZE];
  // for (int i = 0; i < INDEX_SIZE; i++)
  // {
  //   index[i] = (ESimOne_Data_Vehicle_State)i;
  // }
  // for (int i=0; i<1000; i++)
  // {
  //   if (!SimOneAPI::RegisterVehicleState("0", index, INDEX_SIZE))
  //   {
  //     printf("\033[1m\033[31mSimOneAPI::RegisterVehicleState Faile!\033[0m\n");
  //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
  //     continue;
  //   }
  //   break;
  // }

	if (!SimOneAPI::GetCaseInfo(&mCaseInfo))
  {
    printf("\033[1m\033[31mSimOneAPI::GetCaseInfo Failed!\033[0m\n");
  }

  return true;
}

bool Trans::PubIni()
{
    if (!mpPubGps->init(mTransport, mKind, mTopicGps, mDomain, EPub_Gps, mPub_TCP_PortBase))
    {
        printf("\033[1m\033[31m[PubIni] Pub Gps Init Failed!\033[0m\n");
        return false;
    }
    if (!mpPubObstacle->init(mTransport, mKind, mTopicObstacle, mDomain, EPub_Obstacle, mPub_TCP_PortBase))
    {
        printf("\033[1m\033[31m[PubIni] Pub Obstacle Init Failed!\033[0m\n");
        return false;
    }
#ifdef DEBUG_CONTROL
    if (!mpPubControl->init(mTransport, mKind, mTopicControl, mDomain, EPub_Control, mPub_TCP_PortBase))
    {
        printf("\033[1m\033[31m[PubIni] Pub Control Init Failed!\033[0m\n");
        return false;
    }
#endif
    return true;
}

bool Trans::SubIni()
{
    if (!mpSubControl->init(mTransport, mKind, mTopicControl, mDomain, ESub_Control, mSub_TCP_Ip, mSub_TCP_PortBase ))
    {
        printf("\033[1m\033[31m[PubIni] Sub Control Init Failed!\033[0m\n");
        return false;
    }
    mpSubControl->setDriveTask(&SimOneAPI::SetDrive);

    return true;
}

bool Trans::PramOpt(int argc, char *argv[])
{
  if (argc == 2)
  {
    if (common::verify_ip::validateIP(argv[1]))
    {
      mBridgeIO_Ip = argv[1];
    }
    else
    {
      printf("\033[1m\033[31m[pram_opt] Invalid IP Address!\033[0m\n");
      return false;
    }
  }

  return true;
}

bool Trans::Init()
{
    ConfigIni();
    if (!SimOneIni())
    {
        return false;
    }
    if (!PubIni())
    {
        return false;
    }
    if (!SubIni())
    {
        return false;
    }
    return true;
}

void Trans::CB_PubGps(const char *mainVehicleId, SimOne_Data_Gps *pGps)
{
    mpPubGps->publish(static_cast<void *>(pGps));
#ifdef DEBUG_CONTROL
    static uint32_t i = 0;
    if (i > 100)
    {
        mpPubControl->publish(nullptr);
        printf("-------------- published\n");
        i = 0;
    }
    i++;
#endif
}

void Trans::CB_PubObstacle(const char* mainVehicleId, SimOne_Data_Obstacle *pObstacle)
{
    mpPubObstacle->publish(static_cast<void *>(pObstacle));
}

void Trans::Pub()
{
    if (!SimOneAPI::SetGpsUpdateCB(CB_PubGps))
    {
        printf("\033[1m\033[31m[run_pub]: SetSimOneGpsCB Failed!\033[0m\n");
    }
    if (!SimOneAPI::SetGroundTruthUpdateCB(CB_PubObstacle))
    {
        printf("\033[1m\033[31m[run_pub]: SetSimOneGroundTruthCB Failed!\033[0m\n");
    }
    // if (enable_physical_sensor)
    // {
    //     if (!SimOneAPI::SetStreamingImageUpdateCB(img_ip.c_str(), img_port, pub_image_cb))
    //     {
    //         printf("\033[1m\033[31m[run_pub]: SetImageUpdateCB Failed!\033[0m\n");
    //     }
    //     if (!SimOneAPI::SetStreamingPointCloudUpdateCB(pcd_ip.c_str(), pcd_port, pcd_port_info, pub_point_cloud_cb))
    //     {
    //         printf("\033[1m\033[31m[run_pub]: SetPointCloudUpdateCB Failed!\033[0m\n");
    //     }
    // }
    // if (!SimOneAPI::SetRadarDetectionsUpdateCB(pub_radar_detections_cb))
    // {
    //     printf("\033[1m\033[31m[run_pub]: SetRadarDetectionsUpdateCB Failed!\033[0m\n");
    // }
    // if (!SimOneAPI::SetSensorDetectionsUpdateCB(pub_sensor_detections_cb))
    // {
    //     printf("\033[1m\033[31m[run_pub]: SetSensorDetectionsUpdateCB Failed!\033[0m\n");
    // }
    // if (!SimOneAPI::SetSensorLaneInfoCB(pub_sensor_laneInfo_cb))
    // {
    //     printf("\033[1m\033[31m[run_pub]: SetSensorLaneInfoCB Failed!\033[0m\n");
    // }
}

void Trans::Sub()
{
    mpSubControl->run();
}

void Trans::MonitorCaseStatus()
{
  while (true)
  {
    if(SimOneAPI::GetCaseRunStatus() ==ESimOne_Case_Status::ESimOne_Case_Status_Stop)
    {
      SimOneAPI::TerminateSimOneAPI();
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  }
}

void Trans::Run()
{
    std::thread pub_thread = std::thread(std::bind(&Trans::Pub, this));
    std::thread sub_thread = std::thread(std::bind(&Trans::Sub, this));
    pub_thread.detach();
    sub_thread.detach();
    MonitorCaseStatus();
}
