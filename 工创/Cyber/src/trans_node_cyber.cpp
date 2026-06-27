#include "trans_node_cyber.hpp"

bool cyber_trans_node::mSigTerm = false;
std::ofstream cyber_trans_node::log_img;
std::string cyber_trans_node::bio_ip;
std::string cyber_trans_node::vehicle_id;
bool cyber_trans_node::enable_physical_sensor;
// PCD_Param_T cyber_trans_node::mPCDParam;
// Img_Param_T cyber_trans_node::mImgParam;

std::string cyber_trans_node::mChannelControl_R;
std::string cyber_trans_node::mChannelEstimate_R;
std::string cyber_trans_node::mChannelRoutingResponse_R;
std::string cyber_trans_node::mChannelTrajectory_R;
std::string cyber_trans_node::mChannelChassis_W;
std::string cyber_trans_node::mChannelContiRadar_W;
std::string cyber_trans_node::mChannelGps_W;
std::string cyber_trans_node::mChannelIMU_W;
std::string cyber_trans_node::mChannelStat_W;
std::string cyber_trans_node::mChannelObstacles_W;
std::string cyber_trans_node::mChannelRoutingReq_W;
std::string cyber_trans_node::mChannelTrafficLight_W;
// std::string cyber_trans_node::mChannelImage_W;
// std::string cyber_trans_node::mChannelPointCloud_W;

cyber_trans_node::cyber_trans_node():
  mChassisImu(mChannelChassis_W, mChannelIMU_W),
  mGps(mChannelGps_W, mChannelStat_W),
  mChassisImuGps(mChannelChassis_W, mChannelIMU_W, mChannelGps_W, mChannelStat_W),
  mContiRadar(mChannelContiRadar_W),
  mCtl(mChannelControl_R),
  mObstacles(mChannelObstacles_W),
  mRoutingReq(mChannelRoutingReq_W),
  mTrafficLight(mChannelTrafficLight_W),
  mTrajectory(mChannelTrajectory_R)
  // mImage(mChannelImage_W),
  // mPointCloud(mChannelPointCloud_W)
{
  exit_ini();
  mSigTerm = false;
}
cyber_trans_node::~cyber_trans_node(){}

int64_t cyber_trans_node::getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000; // milliseconds
    // return tv.tv_sec * 1e6 + tv.tv_usec; // microseconds
}

// void cyber_trans_node::run_pub()
// {
//   Timer timer_chassis_imu, timer_conti_radar, timer_gps, timer_obstacles, timer_routing_req, timer_traffic_light;
//   bool routing_rsp = false;
// 
//   timer_chassis_imu.start(16, std::bind(&CW_Chassis_Imu::task_chassis_imu, &mChassisImu));
//   // timer_conti_radar.start(16, std::bind(&CW_ContiRadar::task_conti_radar, &mContiRadar));
//   timer_gps.start(16, std::bind(&CW_Gps::task_gps, &mGps));
//   timer_obstacles.start(16, std::bind(&CW_Obstacles::task_obstacles, &mObstacles));
//   timer_routing_req.start(32, std::bind(&CW_RoutingReq::task_routing_req, &mRoutingReq, routing_rsp));
//   timer_traffic_light.start(16, std::bind(&CW_TrafficLight::task_traffic_light, &mTrafficLight));
// 
//   if (enable_physical_sensor)
//   {
//     mPointCloud.task_point_cloud();
//     mImage.task_image();
//   }
// 
//   while (true)
//   {
//     if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop || mSigTerm)
//     {
//       timer_chassis_imu.stop();
//       // timer_conti_radar.stop();
//       timer_gps.stop();
//       timer_obstacles.stop();
//       timer_routing_req.stop();
//       timer_traffic_light.stop();
// 
//       std::cout << "Terminate SimOne API ..." << std::endl;
//       if (!SimOneAPI::TerminateSimOneAPI())
//       {
//         std::cout << "SimOneAPI TerminateSimOneAPI Failed!" << std::endl;
//       }
//       break;
//     }
// 
//     if(routing_rsp)
//     {
//       timer_routing_req.stop();
//     }
//     
//     std::this_thread::sleep_for(std::chrono::seconds(3));
//   }
// }

void cyber_trans_node::run_pub()
{
  Timer timer_routing_req, timer_traffic_light, timer_null_obstacles;
  bool routing_rsp = false;
  
  mChassisImuGps.task_chassis_imu_gps(); // mChassisImu.task_chassis_imu(); // mGps.task_gps();
  // mContiRadar.task_conti_radar();
  mObstacles.task_obstacles();
  timer_routing_req.start(16, std::bind(&CW_RoutingReq::task_routing_req, &mRoutingReq, routing_rsp));
  timer_traffic_light.start(16, std::bind(&CW_TrafficLight::task_traffic_light, &mTrafficLight));
  timer_null_obstacles.start(16, std::bind(&CW_Obstacles::set_null_obstacles, &mObstacles));

  // if (enable_physical_sensor)
  // {
  //   mImage.task_image();
  //   mPointCloud.task_point_cloud();
  // }

  while (true)
  {
    if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop || mSigTerm)
    {
      timer_traffic_light.stop();
      timer_null_obstacles.stop();

      // std::cout << "Terminate SimOne API ..." << std::endl;
      // if (!SimOneAPI::TerminateSimOneAPI())
      // {
      //   std::cout << "SimOneAPI TerminateSimOneAPI Failed!" << std::endl;
      // }
      break;
    }

    if (routing_rsp)
    {
      timer_routing_req.stop();
    }

    std::this_thread::sleep_for(std::chrono::seconds(3));
  }
}

void cyber_trans_node::run_rcv()
{
  Timer timer_ctl, timer_traj;

  timer_ctl.start(16, std::bind(&CW_Ctl::task_ctl, &mCtl));
  timer_traj.start(32, std::bind(&CW_Trajectory::task_trajectory, &mTrajectory));

  while (true)
  {
    if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop || mSigTerm)
    {
      timer_ctl.stop();
      timer_traj.stop();
      break;
    }

    std::this_thread::sleep_for(std::chrono::seconds(3));
  }
}

void cyber_trans_node::monitor_case_status()
{
  while (true)
  {
    if(SimOneAPI::GetCaseRunStatus() ==ESimOne_Case_Status::ESimOne_Case_Status_Stop)
    {
      std::cout << "Terminate SimOne API ..." << std::endl;
      if (!SimOneAPI::TerminateSimOneAPI())
      {
        std::cout << "SimOneAPI TerminateSimOneAPI Failed!" << std::endl;
      }
      sleep(3);
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  }
}

void cyber_trans_node::exit_handler(int signo)
{
  fprintf(stderr, "**** Ctrl+C,quit ! \n");
  mSigTerm = true;
}
void cyber_trans_node::exit_ini()
{
  struct sigaction sa, osa;
  sa.sa_handler = exit_handler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  if (sigaction(SIGINT, &sa, &osa) < 0)
  {
    std::cout << "Set signal ctrl+c failure!" << std::endl;
  }
}
void cyber_trans_node::config_ini()
{
	rr::RrConfig config;
	config.ReadConfig("config.ini");
  if (bio_ip.empty())
  {
	  bio_ip = config.ReadString("BridgeIO", "BridgeIO_IP", "");
  }

  vehicle_id = config.ReadString("HostVehicle", "Vehicle_ID", "0");

  enable_physical_sensor = config.ReadInt("Sensor", "ENABLED", 0) > 0;

  // mImgParam.ip = config.ReadString("Sensor", "IMG_IP", "127.0.0.1");
  // mImgParam.port = config.ReadInt("Sensor", "IMG_PORT", 0);

  // mPCDParam.ip = config.ReadString("Sensor", "PCD_IP", "127.0.0.1");
  // mPCDParam.port = config.ReadInt("Sensor", "PCD_PORT", 0);
  // mPCDParam.infoPort = config.ReadInt("Sensor", "PCD_PORT_INFO", 0);
  // mPCDParam.frameId = config.ReadString("Sensor", "PCD_FRAMEID", "velodyne128");

  mChannelControl_R = config.ReadString("CYBER", "R_CONTROL_CHANNEL", "");
  mChannelEstimate_R = config.ReadString("CYBER", "R_ESTIMATE_CHANNEL", "");
  mChannelRoutingResponse_R = config.ReadString("CYBER", "R_ROUTING_RESPONSE_CHANNEL", "");
  mChannelTrajectory_R = config.ReadString("CYBER", "R_TRAJECTORY_CHANNEL", "");
  mChannelChassis_W = config.ReadString("CYBER", "W_CHASSIS_CHANNEL", "");
  mChannelContiRadar_W = config.ReadString("CYBER", "W_CONTI_RADAR_CHANNEL", "");
  mChannelGps_W = config.ReadString("CYBER", "W_GPS_CHANNEL", "");
  mChannelIMU_W = config.ReadString("CYBER", "W_IMU_CHANNEL", "");
  mChannelStat_W = config.ReadString("CYBER", "W_STAT_CHANNEL", "");
  mChannelObstacles_W = config.ReadString("CYBER", "W_OBSTACLES_CHANNEL", "");
  mChannelRoutingReq_W = config.ReadString("CYBER", "W_ROUTING_REQ_CHANNEL", "");
  mChannelTrafficLight_W = config.ReadString("CYBER", "W_TRAFFICLIGHT_CHANNEL", "");
  // mChannelImage_W = config.ReadString("CYBER", "W_IMAGE_CHANNEL", "");
  // mChannelPointCloud_W = config.ReadString("CYBER", "W_POINT_CLOUD_CHANNEL", "");

	// float lat = config.ReadFloat("MapBase", "BASE_LATITUDE", 0);
	// float log = config.ReadFloat("MapBase", "BASE_LONGITUDE", 0);
	// float alt = config.ReadFloat("MapBase", "BASE_ALTITUDE", 0);
}

void cyber_trans_node::simone_ini()
{
  printf("Connecting BridgeIO With Ip : %s ...\n", bio_ip.c_str());

  if (!SimOneAPI::InitSimOneAPI(vehicle_id.c_str(), false, bio_ip.c_str()))
  {
    printf("[simone_ini] SimOneAPI::InitSimOneAPI Failed!\n");
    return;
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

	if (!SimOneAPI::GetCaseInfo(&caseInfo))
  {
    printf("\033[1m\033[31mSimOneAPI::GetCaseInfo Failed!\033[0m\n");
  }
  // ------------ RegisterSimOneVehicleState End ------------
}
void cyber_trans_node::init()
{
  // mPointCloud.set_params(mPCDParam);
  // mImage.set_params(mImgParam);

  simone_ini();
}

void cyber_trans_node::run_autopilot()
{
  std::unique_ptr<SimOne_Data_Map> pMap = std::make_unique<SimOne_Data_Map>();
  if (!SimOneAPI::GetHDMapData(pMap.get()))
  {
    printf("\033[1m\033[31m[conf_apollo_map] SimOneAPI::GetHDMapData Failed!\033[0m\n");
    return;
  }
  std::string simone_od = "--od=/tmp/";
  simone_od += pMap->openDrive;
  simone_od += "/";
  simone_od += pMap->opendriveMd5;
	simone_od += "/base_map.xodr";
  struct stat buf;
  if (stat(simone_od.c_str(), &buf) && buf.st_size > 10)
  {
    printf("\033[1m\033[31m[conf_apollo_map] %s not exist!\033[0m\n", simone_od.c_str());
    return;
  }

  int ret;
  if (fork() == 0)
  {
    if (execlp("python", "python", "./autopilot.py", simone_od.c_str(), NULL) == -1)
    {
      printf("\033[1m\033[31m[run_apollo] execlp Failed!\033[0m\n");
      return;
    }
  }
  else
  {
    std::string cmd_kill_apollo = "-k";
    // std::string cmd_launch_apollo = "--url=" + mOpenDriveURL + " --md5=" + mOpenDriveMd5 + "--loop=0 --scene_name=" + caseInfo.caseName;
    wait(&ret);
    while (true)
    {

      if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop || mSigTerm)
      {
        if (fork() == 0)
        {
          if (execlp("python", "python", "./autopilot.py", cmd_kill_apollo.c_str(), NULL) == -1)
          {
            printf("\033[1m\033[31m[run_apollo] execlp Failed!\033[0m\n");
            return;
          }
        }
        else
        {
          wait(&ret);
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }
  }

  // Py_Initialize();
  // if (!Py_IsInitialized())
  // {
  //   printf("\033[1m\033[31m[run_apollo] Py_Initialize Failed!\033[0m\n");
  //   return;
  // }

  // PyObject *pModule = NULL;
  // PyObject *pFunc = NULL;
  // PyObject *pCall = NULL;
  // PyRun_SimpleString("import sys");
  // PyRun_SimpleString("sys.path.append('./')");
  // pModule = PyImport_ImportModule("preStart");
  // if (pModule == NULL)
  // {
  //   PyErr_Print();
  //   printf("\033[1m\033[31m[run_apollo] PyImport_ImportModule Failed!\033[0m\n");
  //   return;
  // }
  // pFunc = PyObject_GetAttrString(pModule, "main");
  // if (pFunc == NULL)
  // {
  //   PyErr_Print();
  //   printf("\033[1m\033[31m[run_apollo] PyObject_GetAttrString Failed!\033[0m\n");
  //   return;
  // }

  // std::string cmd_kill_apollo = "preStart.py -k";
  // PyObject *pArgKill = PyTuple_New(1);
  // PyTuple_SetItem(pArgKill, 0, Py_BuildValue("s", cmd_kill_apollo.c_str()));
  // pCall = PyObject_CallFunctionObjArgs(pFunc, pArgKill);
  // if (!pCall || PyCallable_Check(pCall))
  // {
  //   PyErr_Print();
  //   printf("\033[1m\033[31m[run_apollo] PyEval_CallObject Failed!\033[0m\n");
  // }

  // std::string cmd_launch_apollo = "preStart.py --url" + mOpenDriveURL + " --md5=" + mOpenDriveMd5 + "--loop=0 --scene_name=" + caseInfo.caseName;
  // PyObject *pArgLaunch = PyTuple_New(1);
  // PyTuple_SetItem(pArgLaunch, 0, Py_BuildValue("s", cmd_launch_apollo.c_str()));
  // pCall = PyObject_CallFunctionObjArgs(pFunc, pArgLaunch);
  // if (!pCall || PyCallable_Check(pCall))
  // {
  //   PyErr_Print();
  //   printf("\033[1m\033[31m[run_apollo] PyEval_CallObject Failed!\033[0m\n");
  // }

  // Py_Finalize();
}

void cyber_trans_node::run()
{
	init();
	std::thread apollo_thread = std::thread(std::bind(&cyber_trans_node::run_autopilot, this));
	std::thread pub_thread = std::thread(std::bind(&cyber_trans_node::run_pub, this));
	std::thread rcv_thread = std::thread(std::bind(&cyber_trans_node::run_rcv, this));
  apollo_thread.join();
  pub_thread.join();
  rcv_thread.join();
}

bool cyber_trans_node::pram_opt(int argc, char *argv[])
{
  if (argc == 2)
  {
    if (common::verify_ip::validateIP(argv[1]))
    {
      bio_ip = argv[1];
    }
    else
    {
      printf("\033[1m\033[31m[pram_opt] Invalid IP Address!\033[0m\n");
      return false;
    }
  }

  return true;
}

int main(int argc, char *argv[])
{
  if (!cyber_trans_node::pram_opt(argc, argv))
  {
    return 0;
  }

  while (true)
  {
    cyber_trans_node::config_ini();
    cyber_trans_node tn;
    tn.run();

    std::cout << "Terminate SimOne API ..." << std::endl;
    if (!SimOneAPI::TerminateSimOneAPI())
    {
      std::cout << "SimOneAPI TerminateSimOneAPI Failed!" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  }
  return 0;
}
